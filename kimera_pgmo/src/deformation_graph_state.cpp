#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <kimera_rpgo/pose_graph_4dof.h>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {
namespace {

bool isDeformation(const gtsam::NonlinearFactor& factor) {
  return dynamic_cast<const DeformationEdgeFactor*>(&factor) ||
         dynamic_cast<const DeformationEdgeFactorToPose4DoF*>(&factor) ||
         dynamic_cast<const DeformationEdgeFactorFromPose4DoF*>(&factor) ||
         dynamic_cast<const DeformationEdgeFactor4DoF*>(&factor);
}

}  // namespace

void OptimizationState::filter(
    const std::function<bool(const gtsam::NonlinearFactor&)>& keep) {
  OptimizationState filtered;
  if (!inlier_weights.empty() && inlier_weights.size() != factors.size()) {
    throw std::logic_error("Factor weights are not aligned");
  }

  for (size_t i = 0; i < factors.size(); ++i) {
    const auto& factor = factors[i];
    if (!factor || !keep(*factor)) {
      continue;
    }

    if (known_inliers.count(i)) {
      filtered.known_inliers.insert(filtered.factors.size());
    }

    if (!inlier_weights.empty()) {
      filtered.inlier_weights.push_back(inlier_weights[i]);
    }

    filtered.factors.add(factor);
  }

  factors = std::move(filtered.factors);
  known_inliers = std::move(filtered.known_inliers);
  inlier_weights = std::move(filtered.inlier_weights);
}

bool DeformationGraph::isPose4(gtsam::Key key) const {
  return pose_mode_ == PoseMode::POSE4DOF && !mesh_keys_.count(key);
}

const gtsam::Pose3& DeformationGraph::originalPose(gtsam::Key key) const {
  return original_poses_.at(key);
}

void DeformationGraph::insertValue(gtsam::Key key,
                                   const gtsam::Pose3& pose,
                                   bool temp) {
  auto& state = temp ? temporary_ : permanent_;
  if (permanent_.values.exists(key) || temporary_.values.exists(key)) {
    throw std::invalid_argument("Duplicate graph key");
  }

  original_poses_.emplace(key, pose);
  if (temp) {
    temp_pg_initial_poses_.emplace(key, pose);
  }

  if (isPose4(key)) {
    state.values.insert(key, gtsam::Pose4DoF(pose));
  } else {
    state.values.insert(key, pose);
  }

  (temp ? temp_values_ : values_)->insert(key, pose);
}

void DeformationGraph::updateState(const gtsam::Values& updates, bool temp) {
  if (updates.empty()) {
    return;
  }

  auto& state = temp ? temporary_ : permanent_;
  gtsam::Values candidate;
  const auto poses = kimera_rpgo::pose3Estimates(updates);
  for (const auto& entry : poses) {
    if (!state.values.exists(entry.key)) {
      throw std::invalid_argument("Estimate update references a missing graph node");
    }

    const auto& pose = entry.value.cast<gtsam::Pose3>();
    if (isPose4(entry.key)) {
      const auto original = gtsam::Pose4DoF(originalPose(entry.key));
      candidate.insert(entry.key,
                       gtsam::Pose4DoF(pose.x(),
                                       pose.y(),
                                       pose.z(),
                                       pose.rotation().yaw(),
                                       original.pitch(),
                                       original.roll()));
    } else {
      candidate.insert(entry.key, pose);
    }
  }

  state.values.update(candidate);
  (temp ? temp_values_ : values_)->update(kimera_rpgo::pose3Estimates(candidate));
  clearMeshCache();
  recalculate_vertices_ = true;
}

void DeformationGraph::refreshEstimates() {
  *values_ = kimera_rpgo::pose3Estimates(permanent_.values);
  *temp_values_ = kimera_rpgo::pose3Estimates(temporary_.values);
}

void DeformationGraph::appendFactor(const gtsam::NonlinearFactor::shared_ptr& factor,
                                    bool temp,
                                    bool known_inlier) {
  auto& state = temp ? temporary_ : permanent_;
  for (const auto key : factor->keys()) {
    if (!permanent_.values.exists(key) && !(temp && temporary_.values.exists(key))) {
      throw std::invalid_argument("Factor references a missing graph node");
    }
  }

  if (known_inlier) {
    state.known_inliers.insert(state.factors.size());
  }

  state.factors.add(factor);
  state.inlier_weights.clear();
}

gtsam::NonlinearFactor::shared_ptr DeformationGraph::makePrior(gtsam::Key key,
                                                               const gtsam::Pose3& pose,
                                                               double variance) const {
  if (isPose4(key)) {
    const auto variances =
        (gtsam::Vector4() << variance, variance, variance, 0.01 * variance).finished();
    return boost::make_shared<gtsam::PriorFactor<gtsam::Pose4DoF>>(
        key, gtsam::Pose4DoF(pose), gtsam::noiseModel::Diagonal::Variances(variances));
  }

  gtsam::Vector6 variances;
  variances.head<3>().setConstant(0.01 * variance);
  variances.tail<3>().setConstant(variance);
  return boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      key, pose, gtsam::noiseModel::Diagonal::Variances(variances));
}

gtsam::NonlinearFactor::shared_ptr DeformationGraph::makePrior(
    gtsam::Key key,
    const gtsam::Pose3& pose,
    const gtsam::SharedNoiseModel& noise) const {
  if (!noise || noise->dim() != 6) {
    throw std::invalid_argument(
        "Pose3 prior measurements require six-dimensional noise");
  }

  if (isPose4(key)) {
    return boost::make_shared<gtsam::PriorFactor<gtsam::Pose4DoF>>(
        key,
        gtsam::Pose4DoF(pose),
        kimera_rpgo::projectBetweenNoise(gtsam::Pose3(), pose, noise));
  }

  return boost::make_shared<gtsam::PriorFactor<gtsam::Pose3>>(key, pose, noise);
}

gtsam::NonlinearFactor::shared_ptr DeformationGraph::makeDeformationFactor(
    gtsam::Key from,
    gtsam::Key to,
    const gtsam::Point3& measurement,
    const gtsam::SharedNoiseModel& noise) const {
  if (!noise || noise->dim() != 3) {
    throw std::invalid_argument("Deformation edges require three-dimensional noise");
  }

  if (isPose4(from) && isPose4(to)) {
    return boost::make_shared<DeformationEdgeFactor4DoF>(from, to, measurement, noise);
  }

  if (isPose4(from)) {
    return boost::make_shared<DeformationEdgeFactorFromPose4DoF>(
        from, to, measurement, noise);
  }

  if (isPose4(to)) {
    return boost::make_shared<DeformationEdgeFactorToPose4DoF>(
        from, to, measurement, noise);
  }

  return boost::make_shared<DeformationEdgeFactor>(from, to, measurement, noise);
}

OptimizationSnapshot DeformationGraph::optimizationSnapshot() const {
  return {permanent_, temporary_};
}

void DeformationGraph::setPoseTimestamp(gtsam::Key key, Timestamp stamp) {
  if (!original_poses_.count(key) || mesh_keys_.count(key)) {
    throw std::invalid_argument("Timestamp requires an existing pose node");
  }

  pose_stamps_[key] = stamp;
}

std::optional<Timestamp> DeformationGraph::getPoseTimestamp(gtsam::Key key) const {
  const auto iter = pose_stamps_.find(key);
  return iter == pose_stamps_.end() ? std::nullopt
                                    : std::optional<Timestamp>(iter->second);
}

std::map<size_t, std::vector<Timestamp>> DeformationGraph::getPoseTimestamps() const {
  std::map<size_t, std::vector<Timestamp>> result;
  for (const auto& [prefix, poses] : pg_initial_poses_) {
    const auto robot = robot_prefix_to_id.find(prefix);
    if (robot == robot_prefix_to_id.end()) {
      continue;
    }

    auto& stamps = result[robot->second];
    for (size_t i = 0; i < poses.size(); ++i) {
      const auto iter = pose_stamps_.find(gtsam::Symbol(prefix, i));
      if (iter == pose_stamps_.end()) {
        throw std::runtime_error(
            "Pose timestamps are incomplete; supply migration timestamps");
      }

      stamps.push_back(iter->second);
    }
  }

  return result;
}

void DeformationGraph::rebuildBookkeeping() {
  adjacency_map_.clear();
  num_loopclosures_ = 0;
  for (const auto& factor : permanent_.factors) {
    if (isDeformation(*factor)) {
      adjacency_map_[factor->front()].insert(factor->back());
    }

    const bool between =
        dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(factor.get()) ||
        dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose4DoF>*>(factor.get());
    if (between && factor->back() != factor->front() + 1) {
      ++num_loopclosures_;
    }
  }

  for (const auto& factor : temporary_.factors) {
    if (isDeformation(*factor)) {
      adjacency_map_[factor->front()].insert(factor->back());
    }
  }

  clearMeshCache();
  recalculate_vertices_ = true;
}

void DeformationGraph::clear() {
  permanent_ = {};
  temporary_ = {};
  values_->clear();
  temp_values_->clear();
  original_poses_.clear();
  mesh_keys_.clear();
  pose_stamps_.clear();
  pg_initial_poses_.clear();
  temp_pg_initial_poses_.clear();
  vertex_positions_.clear();
  vertex_stamps_.clear();
  rebuildBookkeeping();
}

void DeformationGraph::clearFactors() {
  permanent_.filter([](const auto&) { return false; });
  temporary_.filter([](const auto&) { return false; });
  rebuildBookkeeping();
}

void DeformationGraph::clearTemporaryStructures() {
  for (const auto key : temporary_.values.keys()) {
    original_poses_.erase(key);
    pose_stamps_.erase(key);
  }

  temporary_ = {};
  temp_values_->clear();
  temp_pg_initial_poses_.clear();
  rebuildBookkeeping();
}

void DeformationGraph::clearMeshEdgeFactorsOnly() {
  const auto keep = [&](const auto& factor) {
    for (const auto key : factor.keys()) {
      if (mesh_keys_.count(key)) {
        return false;
      }
    }

    return true;
  };
  permanent_.filter(keep);
  temporary_.filter(keep);
  rebuildBookkeeping();
}

void DeformationGraph::clearMeshNodesOnly() {
  clearMeshEdgeFactorsOnly();
  for (const auto key : mesh_keys_) {
    permanent_.values.erase(key);
    original_poses_.erase(key);
  }

  mesh_keys_.clear();
  vertex_positions_.clear();
  vertex_stamps_.clear();
  refreshEstimates();
}

void DeformationGraph::removeMeshNodesAbove(char prefix, size_t cutoff_index) {
  const auto keep = [&](const auto& factor) {
    for (const auto key : factor.keys()) {
      const gtsam::Symbol symbol(key);
      if (mesh_keys_.count(key) && symbol.chr() == prefix &&
          symbol.index() >= cutoff_index) {
        return false;
      }
    }

    return true;
  };
  permanent_.filter(keep);
  temporary_.filter(keep);
  const auto iter = vertex_positions_.find(prefix);
  if (iter == vertex_positions_.end()) {
    return;
  }

  const auto size = iter->second.size();
  for (size_t i = cutoff_index; i < size; ++i) {
    const gtsam::Key key = gtsam::Symbol(prefix, i);
    permanent_.values.erase(key);
    original_poses_.erase(key);
    mesh_keys_.erase(key);
  }

  if (cutoff_index == 0) {
    vertex_positions_.erase(iter);
    vertex_stamps_.erase(prefix);
  } else {
    iter->second.resize(std::min(size, cutoff_index));
    vertex_stamps_.at(prefix).resize(iter->second.size());
  }

  refreshEstimates();
  rebuildBookkeeping();
}

void DeformationGraph::reindexMeshNodes(
    char prefix, const std::unordered_map<size_t, size_t>& old_to_new) {
  const auto check = [&](const OptimizationState& state) {
    for (const auto& factor : state.factors) {
      for (const auto key : factor->keys()) {
        if (mesh_keys_.count(key) && gtsam::Symbol(key).chr() == prefix) {
          throw std::invalid_argument("Clear mesh edge factors before reindexing");
        }
      }
    }
  };
  check(permanent_);
  check(temporary_);
  const auto& old_positions = vertex_positions_.at(prefix);
  const auto& old_stamps = vertex_stamps_.at(prefix);
  std::set<size_t> destinations;
  for (const auto& [old_idx, new_idx] : old_to_new) {
    if (old_idx >= old_positions.size() || new_idx >= old_to_new.size() ||
        !destinations.insert(new_idx).second) {
      throw std::invalid_argument(
          "Reindex mapping must have valid sources and dense unique destinations");
    }
  }

  std::vector<gtsam::Point3> positions(old_to_new.size());
  std::vector<Timestamp> stamps(old_to_new.size());
  gtsam::Values values;
  std::map<gtsam::Key, gtsam::Pose3> originals;
  for (const auto& [old_idx, new_idx] : old_to_new) {
    const gtsam::Key old_key = gtsam::Symbol(prefix, old_idx);
    const gtsam::Key new_key = gtsam::Symbol(prefix, new_idx);
    positions[new_idx] = old_positions[old_idx];
    stamps[new_idx] = old_stamps[old_idx];
    values.insert(new_key, permanent_.values.at(old_key));
    originals.emplace(new_key, originalPose(old_key));
  }

  removeMeshNodesAbove(prefix, 0);
  if (!positions.empty()) {
    vertex_positions_[prefix] = std::move(positions);
    vertex_stamps_[prefix] = std::move(stamps);
  }

  for (const auto& entry : values) {
    permanent_.values.insert(entry.key, entry.value);
    mesh_keys_.insert(entry.key);
  }

  original_poses_.insert(originals.begin(), originals.end());
  refreshEstimates();
  rebuildBookkeeping();
}

void DeformationGraph::processPointMeasurement(gtsam::Key from_key,
                                               gtsam::Key to_key,
                                               const gtsam::Pose3& from_pose,
                                               const gtsam::Point3& to_point,
                                               const gtsam::SharedNoiseModel& noise,
                                               bool temp,
                                               bool known_inlier) {
  if (!values_->exists(to_key) && !temp_values_->exists(to_key)) {
    insertValue(to_key, gtsam::Pose3(gtsam::Rot3(), to_point), temp);
  }

  appendFactor(
      makeDeformationFactor(from_key, to_key, from_pose.transformTo(to_point), noise),
      temp,
      known_inlier);
  adjacency_map_[from_key].insert(to_key);
}

void DeformationGraph::processNewMeshEdgesAndNodes(
    const std::vector<std::pair<gtsam::Key, gtsam::Key>>& mesh_edges,
    const gtsam::Values& mesh_nodes,
    const std::unordered_map<gtsam::Key, Timestamp>& node_stamps,
    std::vector<size_t>* added_indices,
    std::vector<Timestamp>* added_index_stamps,
    const std::vector<double>& edge_variances) {
  if (mesh_edges.size() != edge_variances.size()) {
    throw std::invalid_argument("Each mesh edge requires a variance");
  }

  processNewMeshEdgesAndNodes(
      {}, mesh_nodes, node_stamps, added_indices, added_index_stamps);
  for (size_t i = 0; i < mesh_edges.size(); ++i) {
    const auto& [from, to] = mesh_edges[i];
    if (!checkNewMeshEdge(from, to)) {
      throw std::invalid_argument("Invalid mesh edge endpoint");
    }

    addDeformationEdge(from,
                       to,
                       originalPose(from),
                       originalPose(to).translation(),
                       edge_variances[i],
                       false,
                       true);
  }
}

}  // namespace kimera_pgmo
