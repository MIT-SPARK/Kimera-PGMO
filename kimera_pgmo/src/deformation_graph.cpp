/**
 * @file   deformation_graph.cpp
 * @brief  DeformationGraph class
 * @author Yun Chang
 */
#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include "kimera_pgmo/deformation_graph.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>

#include <algorithm>
#include <cmath>
#include <numeric>

#include "kimera_pgmo/deformation_graph_factor.h"
#include "kimera_pgmo/pcl_mesh_traits.h"

using pcl::PolygonMesh;

namespace kimera_pgmo {
namespace {

gtsam::Symbol rekeyRobotId(gtsam::Symbol key, size_t robot_id) {
  const auto prefix = key.chr();
  if (robot_prefix_to_id.count(prefix)) {
    return gtsam::Symbol(GetRobotPrefix(robot_id), key.index());
  }

  if (vertex_prefix_to_id.count(prefix)) {
    return gtsam::Symbol(GetVertexPrefix(robot_id), key.index());
  }

  return key;
}

}  // namespace

using MeasurementVector = std::vector<std::pair<gtsam::Key, gtsam::Pose3>>;
using RobotTimestampMap = std::map<size_t, std::vector<Timestamp>>;
using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;
using PosePrior = gtsam::PriorFactor<gtsam::Pose3>;
using EdgeType = pose_graph_tools::PoseGraphEdge::Type;
using deformation::DeformationVertex;

void PGOInfo::clear() {
  values.clear();
  factors.resize(0);
  known_inliers.clear();
  inlier_weights.clear();
}

void PGOInfo::rekey(const std::map<char, char>& prefix_map) {
  rekey([&](gtsam::Symbol key) {
    const auto iter = prefix_map.find(key.chr());
    return iter != prefix_map.end() ? gtsam::Symbol(iter->second, key.index()) : key;
  });
}

void PGOInfo::rekey(const std::function<gtsam::Symbol(gtsam::Symbol)>& key_remap) {
  gtsam::Values new_values;
  std::map<gtsam::Key, gtsam::Key> remapping;
  for (const auto& key_value_pair : values) {
    gtsam::Symbol key = key_remap(key_value_pair.key);
    remapping[key_value_pair.key] = key;
    new_values.insert(key, key_value_pair.value);
  }

  values = new_values;
  factors = factors.rekey(remapping);
}

void PGOInfo::forceRobotId(size_t robot_id) {
  rekey([&](gtsam::Symbol key) { return rekeyRobotId(key, robot_id); });
}

DeformationGraph::DeformationGraph(bool add_init_vertex_prior)
    : add_init_vertex_prior_(add_init_vertex_prior),
      num_loopclosures_(0),
      recalculate_vertices_(false) {}

DeformationGraph::~DeformationGraph() {}

size_t DeformationGraph::getNumLoopclosures() const { return num_loopclosures_; }

size_t DeformationGraph::getNumVertices() const {
  size_t num_vertices = 0;
  for (const auto& [prefix, vertices] : vertices_) {
    num_vertices += vertices.size();
  }

  return num_vertices;
}

const gtsam::Values* DeformationGraph::getValues() const { return &info_.values; }

gtsam::Values DeformationGraph::getValuesCopy() const { return info_.values; }

const gtsam::NonlinearFactorGraph* DeformationGraph::getFactors() const {
  return &info_.factors;
}

gtsam::NonlinearFactorGraph DeformationGraph::getFactorsCopy() const {
  return info_.factors;
}

const std::set<size_t>* DeformationGraph::getKnownInlierSet() const {
  return &info_.known_inliers;
}

std::set<size_t> DeformationGraph::getKnownInlierSetCopy() const {
  return info_.known_inliers;
}

const gtsam::Values* DeformationGraph::getTempValues() const {
  return &temp_info_.values;
}

gtsam::Values DeformationGraph::getTempValuesCopy() const { return temp_info_.values; }

const gtsam::NonlinearFactorGraph* DeformationGraph::getTempFactors() const {
  return &temp_info_.factors;
}

gtsam::NonlinearFactorGraph DeformationGraph::getTempFactorsCopy() const {
  return temp_info_.factors;
}

const std::set<size_t>* DeformationGraph::getTempKnownInlierSet() const {
  return &temp_info_.known_inliers;
}

std::set<size_t> DeformationGraph::getTempKnownInlierSetCopy() const {
  return temp_info_.known_inliers;
}

const std::vector<double>* DeformationGraph::getInlierWeights() const {
  return &info_.inlier_weights;
}

const std::vector<double>* DeformationGraph::getTempInlierWeights() const {
  return &temp_info_.inlier_weights;
}

gtsam::Pose3 DeformationGraph::getInitialPose(char prefix, size_t index) const {
  return pg_initial_poses_.at(prefix).at(index);
}

gtsam::Point3 DeformationGraph::getInitialPositionVertex(char prefix,
                                                         size_t index) const {
  return vertices_.at(prefix).at(index).position;
}

bool DeformationGraph::hasVertexKey(char prefix) const {
  return vertices_.count(prefix);
}

bool DeformationGraph::getRecalculateVertices() { return recalculate_vertices_; }

void DeformationGraph::setRecalculateVertices() { recalculate_vertices_ = true; }

void DeformationGraph::clear() {
  info_.clear();
  temp_info_.clear();
  pg_initial_poses_.clear();
  temp_pg_initial_poses_.clear();
  vertices_.clear();
}

void DeformationGraph::clearFactors() {
  info_.factors.resize(0);
  temp_info_.factors.resize(0);
}

void DeformationGraph::clearTemporaryStructures() {
  temp_info_.clear();
  temp_pg_initial_poses_.clear();
}

void DeformationGraph::forceRobotId(size_t robot_id) {
  info_.forceRobotId(robot_id);
  temp_info_.forceRobotId(robot_id);

  const auto old_vertices = vertices_;
  vertices_.clear();
  for (const auto& [prefix, vertices] : old_vertices) {
    const auto new_prefix = GetVertexPrefix(robot_id);
    vertices_[new_prefix] = vertices;
  }

  const auto old_init = pg_initial_poses_;
  pg_initial_poses_.clear();
  for (const auto& [prefix, poses] : old_init) {
    char new_prefix = prefix;
    if (robot_prefix_to_id.count(prefix)) {
      new_prefix = GetRobotPrefix(robot_id);
    }

    if (vertex_prefix_to_id.count(prefix)) {
      new_prefix = GetVertexPrefix(prefix);
    }

    pg_initial_poses_[new_prefix] = poses;
  }

  const auto old_temp_init = temp_pg_initial_poses_;
  temp_pg_initial_poses_.clear();
  for (const auto& [key, pose] : old_temp_init) {
    temp_pg_initial_poses_[rekeyRobotId(key, robot_id)] = pose;
  }
}

bool DeformationGraph::hasPrefixPoses(char prefix) const {
  return pg_initial_poses_.count(prefix);
}

std::unique_lock<std::mutex> DeformationGraph::acquireLock() {
  return std::unique_lock<std::mutex>(mutex_);
}

void DeformationGraph::processPoseGraph(const pose_graph_tools::PoseGraph& pose_graph,
                                        const EdgeTypeVarianceMap& variance_map,
                                        std::map<size_t, size_t> robot_id_remap,
                                        const Eigen::Isometry3d* const transform) {
  for (const auto& node : pose_graph.nodes) {
    auto node_robot_id = getRemappedId(robot_id_remap, node.robot_id);
    auto node_key = gtsam::Symbol(robot_id_to_prefix.at(node_robot_id), node.key);
    gtsam::Pose3 node_pose(node.pose.matrix());
    if (transform) {
      node_pose = gtsam::Pose3(transform->matrix()).compose(node_pose);
    }
    if (checkNewNode(node_key)) {
      addNewNode(node_key, node_pose);
    }
    // Note that the pose here directly updates the pg initial pose. Which matters for
    // the processNodeValence function (connecting node to vertex) but does not matter
    // as much if the node to vertex edges will be directly given
  }

  for (const auto& edge : pose_graph.edges) {
    if (!variance_map.count(edge.type)) {
      SPARK_LOG(ERROR) << "Missing edge type " << edge.type
                       << " for variance map in processPoseGraph";
      continue;
    }
    auto from_robot = getRemappedId(robot_id_remap, edge.robot_from);
    auto to_robot = getRemappedId(robot_id_remap, edge.robot_to);

    auto from_key = gtsam::Symbol(robot_id_to_prefix.at(from_robot), edge.key_from);
    auto to_key = gtsam::Symbol(robot_id_to_prefix.at(to_robot), edge.key_to);
    gtsam::Pose3 from_T_to(edge.pose.matrix());
    if (from_key != to_key) {
      addNewBetween(from_key,
                    to_key,
                    from_T_to,
                    variance_map.at(edge.type),
                    false,
                    edge.type == EdgeType::ODOM);
    } else {
      addPrior(from_key, from_T_to, variance_map.at(edge.type));
    }
  }
}

void DeformationGraph::processMeshGraph(const pose_graph_tools::PoseGraph& mesh_graph,
                                        const EdgeTypeVarianceMap& variance_map,
                                        std::map<size_t, size_t> robot_id_remap,
                                        const Eigen::Isometry3d* const transform) {
  for (const auto& node : mesh_graph.nodes) {
    auto node_robot_id = getRemappedId(robot_id_remap, node.robot_id);
    auto node_key =
        gtsam::Symbol(robot_id_to_vertex_prefix.at(node_robot_id), node.key);
    gtsam::Pose3 node_pose(node.pose.matrix());
    if (transform) {
      node_pose = gtsam::Pose3(transform->matrix()).compose(node_pose);
    }
    addNewMeshNode(node_key, node_pose, node.stamp_ns);
    // Note that the node_pose here directly updates the initial vertex positions. Which
    // matters for the processNodeValence function (connecting node to vertex) but does
    // not matter as much if the node to vertex edges will be directly given. The
    // node_stamp also effects deformation (start index: see deformPoints function)
  }
  for (const auto& edge : mesh_graph.edges) {
    if (!variance_map.count(edge.type)) {
      SPARK_LOG(ERROR) << "Missing edge type " << edge.type
                       << " for variance map in processMeshGraph";
      continue;
    }
    gtsam::Key from_key, to_key;
    auto from_robot = getRemappedId(robot_id_remap, edge.robot_from);
    auto to_robot = getRemappedId(robot_id_remap, edge.robot_to);
    if (edge.type == EdgeType::MESH) {
      from_key = gtsam::Symbol(robot_id_to_vertex_prefix.at(from_robot), edge.key_from);
      to_key = gtsam::Symbol(robot_id_to_vertex_prefix.at(to_robot), edge.key_to);
    } else if (edge.type == EdgeType::POSE_MESH) {
      from_key = gtsam::Symbol(robot_id_to_prefix.at(from_robot), edge.key_from);
      to_key = gtsam::Symbol(robot_id_to_vertex_prefix.at(to_robot), edge.key_to);
    } else if (edge.type == EdgeType::MESH_POSE) {
      from_key = gtsam::Symbol(robot_id_to_vertex_prefix.at(from_robot), edge.key_from);
      to_key = gtsam::Symbol(robot_id_to_prefix.at(to_robot), edge.key_to);
    } else {
      SPARK_LOG(ERROR) << "Unhandled edge type in processMeshGraph";
      continue;
    }

    gtsam::Pose3 from_T_to(edge.pose.matrix());
    addDeformationEdge(from_key,
                       to_key,
                       from_T_to.translation(),
                       variance_map.at(edge.type),
                       false,
                       true);
  }
}

void DeformationGraph::processNodeValence(const gtsam::Key& key,
                                          const Vertices& valences,
                                          char valence_prefix,
                                          double variance,
                                          bool temp) {
  // Add the consistency factors
  for (Vertex v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);

    gtsam::Pose3 node_pose;
    gtsam::Point3 vertex_pos;
    if (!checkNodeValence(key, vertex, node_pose, vertex_pos)) {
      continue;
    }

    addDeformationEdge(key, vertex, node_pose, vertex_pos, variance, temp, true);
    addDeformationEdge(vertex,
                       key,
                       gtsam::Pose3(gtsam::Rot3(), vertex_pos),
                       node_pose.translation(),
                       variance,
                       temp,
                       true);
  }
}

bool DeformationGraph::checkNodeValence(const gtsam::Key& key,
                                        const gtsam::Key& vertex,
                                        gtsam::Pose3& node_pose,
                                        gtsam::Point3& vertex_pos) const {
  if (!info_.values.exists(vertex) && !temp_info_.values.exists(vertex)) {
    return false;
  }

  const auto prefix = gtsam::Symbol(key).chr();
  const auto idx = gtsam::Symbol(key).index();
  const auto valence_prefix = gtsam::Symbol(vertex).chr();
  const auto valence_idx = gtsam::Symbol(vertex).index();
  bool non_temp_node =
      pg_initial_poses_.count(prefix) && pg_initial_poses_.at(prefix).size() > idx;
  node_pose = non_temp_node ? pg_initial_poses_.at(prefix).at(idx)
                            : temp_pg_initial_poses_.at(key);
  vertex_pos = getInitialPositionVertex(valence_prefix, valence_idx);
  return true;
}

void DeformationGraph::processBetweenAsMeshConnections(
    const gtsam::Pose3& w_T_source,
    const Vertices& source,
    const gtsam::Pose3& w_T_dest,
    const Vertices& dest,
    const gtsam::Pose3& source_T_dest,
    char source_prefix,
    char dest_prefix,
    double variance,
    bool temp,
    bool known_inliers) {
  for (const auto& s : source) {
    const gtsam::Symbol vertex_s(source_prefix, s);
    auto w_T_s = gtsam::Pose3(gtsam::Rot3(), vertices_[source_prefix].at(s).position);
    auto s_T_source = w_T_s.between(w_T_source);
    for (const Vertex& d : dest) {
      const gtsam::Symbol vertex_d(dest_prefix, d);
      // Do not add edge to self
      if (vertex_s == vertex_d) {
        continue;
      }
      // For now we do not duplicate deformation graph edges even if there are multiple
      // between factors connecting them. Hence the adjacency book-keeping.
      if (adjacency_map_.count(vertex_s) &&
          adjacency_map_.at(vertex_s).count(vertex_d)) {
        continue;
      }

      auto w_T_d = gtsam::Pose3(gtsam::Rot3(), vertices_[dest_prefix].at(d).position);
      auto d_T_dest = w_T_d.between(w_T_dest);
      auto s_T_d = s_T_source.compose(source_T_dest).compose(d_T_dest.inverse());
      auto d_T_s = s_T_d.inverse();
      addDeformationEdge(
          vertex_s, vertex_d, s_T_d.translation(), variance, temp, known_inliers);
      addDeformationEdge(
          vertex_d, vertex_s, d_T_s.translation(), variance, temp, known_inliers);
    }
  }
}

void DeformationGraph::processPointMeasurement(const gtsam::Key& from_key,
                                               const gtsam::Key& to_key,
                                               const gtsam::Pose3& from_pose,
                                               const gtsam::Point3& to_point,
                                               double variance,
                                               bool temp,
                                               bool known_inlier) {
  if (!info_.values.exists(to_key) && !temp_info_.values.exists(to_key)) {
    if (temp) {
      temp_info_.values.insert(to_key, gtsam::Pose3(gtsam::Rot3(), to_point));
    } else {
      info_.values.insert(to_key, gtsam::Pose3(gtsam::Rot3(), to_point));
    }
  }

  addDeformationEdge(
      from_key, to_key, from_pose, to_point, variance, temp, known_inlier);
}

void DeformationGraph::addDeformationEdge(const gtsam::Key& from_key,
                                          const gtsam::Key& to_key,
                                          const gtsam::Pose3& from_pose,
                                          const gtsam::Point3& to_point,
                                          double variance,
                                          bool temp,
                                          bool known_inlier) {
  // Define noise. Hardcoded for now
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, variance);
  // Create deformation edge factor
  const DeformationEdgeFactor new_edge(from_key, to_key, from_pose, to_point, noise);
  auto& adjacent_keys = adjacency_map_[from_key];
  adjacent_keys.insert(to_key);
  if (temp) {
    if (known_inlier) {
      temp_info_.known_inliers.insert(temp_info_.factors.size());
    }
    temp_info_.factors.add(new_edge);
    return;
  }
  if (known_inlier) {
    info_.known_inliers.insert(info_.factors.size());
  }
  info_.factors.add(new_edge);
}

void DeformationGraph::addDeformationEdge(const gtsam::Key& from_key,
                                          const gtsam::Key& to_key,
                                          const gtsam::Point3& measurement,
                                          double variance,
                                          bool temp,
                                          bool known_inlier) {
  // Define noise. Hardcoded for now
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, variance);
  // Create deformation edge factor
  const DeformationEdgeFactor new_edge(from_key, to_key, measurement, noise);
  auto& adjacent_keys = adjacency_map_[from_key];
  adjacent_keys.insert(to_key);
  if (temp) {
    if (known_inlier) {
      temp_info_.known_inliers.insert(temp_info_.factors.size());
    }
    temp_info_.factors.add(new_edge);
    return;
  }
  if (known_inlier) {
    info_.known_inliers.insert(info_.factors.size());
  }
  info_.factors.add(new_edge);
}

void DeformationGraph::addPrior(const gtsam::Key& key,
                                const gtsam::Pose3& pose,
                                double variance,
                                bool temp,
                                bool known_inlier) {
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  const auto noise = gtsam::noiseModel::Diagonal::Variances(variances);

  gtsam::PriorFactor<gtsam::Pose3> measurement(key, pose, noise);
  if (temp) {
    if (known_inlier) {
      temp_info_.known_inliers.insert(temp_info_.factors.size());
    }
    temp_info_.factors.add(measurement);
  } else {
    if (known_inlier) {
      info_.known_inliers.insert(info_.factors.size());
    }
    info_.factors.add(measurement);
  }
}

void DeformationGraph::processNodeMeasurements(const MeasurementVector& measurements,
                                               double variance) {
  for (auto&& [key, pose] : measurements) {
    if (!info_.values.exists(key)) {
      SPARK_LOG(ERROR) << "DeformationGraph: adding node measurement to a node "
                       << gtsam::DefaultKeyFormatter(key)
                       << " not previously seen before.";
      info_.values.insert(key, pose);
    } else {
      info_.values.update(key, pose);
    }
    addPrior(key, pose, variance);
  }
}

void DeformationGraph::processNewBetween(const gtsam::Key& key_from,
                                         const gtsam::Key& key_to,
                                         const gtsam::Pose3& meas,
                                         double variance) {
  if (!checkNewBetween(key_from, key_to)) {
    return;
  }

  addNewBetween(key_from, key_to, meas, variance);
}

bool DeformationGraph::checkNewBetween(const gtsam::Key& key_from,
                                       const gtsam::Key& key_to) const {
  const auto from_prefix = gtsam::Symbol(key_from).chr();
  const auto to_prefix = gtsam::Symbol(key_to).chr();
  const auto from_idx = gtsam::Symbol(key_from).index();
  const auto to_idx = gtsam::Symbol(key_to).index();

  if (from_idx >= pg_initial_poses_.at(from_prefix).size()) {
    SPARK_LOG(ERROR)
        << "DeformationGraph: when adding new between from key should already exist.";
    return false;
  }

  if (to_idx >= pg_initial_poses_.at(to_prefix).size()) {
    SPARK_LOG(ERROR) << "DeformationGraph: skipping keys in addNewBetween.";
    return false;
  }

  return true;
}

void DeformationGraph::updatePoseGraphInitialGuess(const gtsam::Key& key_from,
                                                   const gtsam::Key& key_to,
                                                   const gtsam::Pose3& meas) {
  const auto to_prefix = gtsam::Symbol(key_to).chr();
  gtsam::Pose3 initial_estimate, init_pose;
  if (info_.values.exists(key_from)) {
    initial_estimate = info_.values.at<gtsam::Pose3>(key_from).compose(meas);
  } else {
    SPARK_LOG(FATAL) << "Missing from key values when adding initial guess.";
  }
  info_.values.insert(key_to, initial_estimate);
  init_pose = pg_initial_poses_[to_prefix].back().compose(meas);
  pg_initial_poses_[to_prefix].push_back(init_pose);
}

void DeformationGraph::addNewBetween(const gtsam::Key& key_from,
                                     const gtsam::Key& key_to,
                                     const gtsam::Pose3& meas,
                                     double variance,
                                     bool temp,
                                     bool known_inlier) {
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);
  if (temp) {
    if (known_inlier) {
      temp_info_.known_inliers.insert(temp_info_.factors.size());
    }
    temp_info_.factors.add(
        gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));
    return;
  }
  if (known_inlier) {
    info_.known_inliers.insert(info_.factors.size());
  }
  info_.factors.add(gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  // if it's a loop closure factor
  if (key_to != key_from + 1) {
    SPARK_LOG(DEBUG) << "DeformationGraph: Added loop closure";
    num_loopclosures_++;
    recalculate_vertices_ = true;
  }
}

void DeformationGraph::processNewTempBetween(const gtsam::Key& key_from,
                                             const gtsam::Key& key_to,
                                             const gtsam::Pose3& meas,
                                             double variance) {
  if (!checkNewTempBetween(key_from, key_to)) {
    return;
  }

  addNewBetween(key_from, key_to, meas, variance, true);
}

bool DeformationGraph::checkNewTempBetween(const gtsam::Key& key_from,
                                           const gtsam::Key& key_to) const {
  if (!info_.values.exists(key_from) && !temp_info_.values.exists(key_from)) {
    SPARK_LOG(ERROR) << "Key does not exist when adding temporary between factor";
    return false;
  }

  if (!info_.values.exists(key_to) && !temp_info_.values.exists(key_to)) {
    SPARK_LOG(ERROR) << "Key does not exist when adding temporary between factor";
    return false;
  }

  return true;
}

void DeformationGraph::processNewTempEdges(const PoseGraph& edges, double variance) {
  for (const auto& e : edges.edges) {
    if (!checkNewTempBetween(e.key_from, e.key_to)) {
      continue;
    }
    addNewBetween(e.key_from, e.key_to, gtsam::Pose3(e.pose.matrix()), variance, true);
  }
  return;
}

bool DeformationGraph::addNewMeshNode(const gtsam::Key& node_key,
                                      const gtsam::Pose3& node_pose,
                                      const Timestamp& node_stamp) {
  const auto prefix = gtsam::Symbol(node_key).chr();
  const auto node_idx = gtsam::Symbol(node_key).index();

  auto iter = vertices_.find(prefix);
  if (iter == vertices_.end()) {
    iter = vertices_.emplace(prefix, std::vector<DeformationVertex>()).first;
  }

  auto& curr_vertices = iter->second;
  if (node_idx != curr_vertices.size()) {
    return false;
  }

  curr_vertices.emplace_back(DeformationVertex{node_stamp, node_pose.translation()});
  // TODO(Yun) temporary hack, check if this assumption always valid even with poses
  if (add_init_vertex_prior_ && info_.values.size() == 0) {
    addPrior(node_key, node_pose, 1e-3, false, true);
  }

  info_.values.insert(node_key, node_pose);
  return true;
}

bool DeformationGraph::checkNewMeshNode(const gtsam::Key& node_key) const {
  char node_prefix = gtsam::Symbol(node_key).chr();
  size_t node_idx = gtsam::Symbol(node_key).index();
  if (!vertices_.count(node_prefix)) {
    if (node_idx == 0) {
      return true;
    }

    return false;
  }

  // The check here returns true even for duplicated (added) nodes
  // Only return false if there is a likely message drop
  return node_idx <= vertices_.at(node_prefix).size();
}

bool DeformationGraph::checkNewMeshEdge(const gtsam::Key& from,
                                        const gtsam::Key& to) const {
  const gtsam::Symbol from_symb(from);
  const gtsam::Symbol to_symb(to);
  if (from_symb.index() >= vertices_.at(from_symb.chr()).size() ||
      to_symb.index() >= vertices_.at(to_symb.chr()).size()) {
    return false;
  }

  if (!info_.values.exists(from) || !info_.values.exists(to)) {
    return false;
  }

  return true;
}

void DeformationGraph::processNewMeshEdgesAndNodes(
    const std::vector<std::pair<gtsam::Key, gtsam::Key>>& mesh_edges,
    const gtsam::Values& mesh_nodes,
    const std::unordered_map<gtsam::Key, Timestamp>& node_stamps,
    std::vector<size_t>* added_indices,
    std::vector<Timestamp>* added_index_stamps,
    double variance) {
  assert(node_stamps.size() == mesh_nodes.size());

  // Iterate and add the new mesh nodes not yet in graph
  // Note that the keys are in increasing order by construction from gtsam
  // Also note that with Hydra we often get duplicates (nodes that have already been
  // added before)
  for (const auto& node_key : mesh_nodes.keys()) {
    if (!checkNewMeshNode(node_key)) {
      SPARK_LOG(FATAL) << "Error adding new mesh node.";
    }

    const gtsam::Pose3& node_pose = mesh_nodes.at<gtsam::Pose3>(node_key);
    if (addNewMeshNode(node_key, node_pose, node_stamps.at(node_key))) {
      added_indices->push_back(gtsam::Symbol(node_key).index());
      added_index_stamps->push_back(node_stamps.at(node_key));
    }
  }

  // Iterate and add the new edges
  for (auto e : mesh_edges) {
    if (!checkNewMeshEdge(e.first, e.second)) {
      SPARK_LOG(FATAL) << "Error adding new mesh edge.";
    }

    const gtsam::Pose3 pose_from = mesh_nodes.at<gtsam::Pose3>(e.first);
    const gtsam::Point3 point_to = mesh_nodes.at<gtsam::Pose3>(e.second).translation();
    addDeformationEdge(e.first, e.second, pose_from, point_to, variance, false, true);
  }
}

void DeformationGraph::processNewNode(const gtsam::Key& key,
                                      const gtsam::Pose3& initial_pose,
                                      bool add_prior,
                                      double prior_variance) {
  if (checkNewNode(key)) {
    addNewNode(key, initial_pose);
  }

  if (add_prior) {
    addPrior(key, initial_pose, prior_variance);
  }
}

bool DeformationGraph::checkNewNode(const gtsam::Key& key) const {
  const auto prefix = gtsam::Symbol(key).chr();
  const auto idx = gtsam::Symbol(key).index();

  if (!pg_initial_poses_.count(prefix) && idx == 0) {
    // First node
    return true;
  }

  if (idx > pg_initial_poses_.at(prefix).size()) {
    SPARK_LOG(ERROR) << "DeformationGraph: Nodes skipped in pose graph nodes";
    return false;
  }

  if (idx < pg_initial_poses_.at(prefix).size()) {
    // Duplicated processing
    return false;
  }
  return true;
}

void DeformationGraph::addNewNode(const gtsam::Key& key,
                                  const gtsam::Pose3& initial_pose) {
  const auto prefix = gtsam::Symbol(key).chr();
  const auto idx = gtsam::Symbol(key).index();
  if (idx == 0) {
    pg_initial_poses_[prefix] = std::vector<gtsam::Pose3>{initial_pose};
  } else {
    pg_initial_poses_[prefix].push_back(initial_pose);
  }

  info_.values.insert(key, initial_pose);
}

void DeformationGraph::processNewTempNode(const gtsam::Key& key,
                                          const gtsam::Pose3& initial_pose,
                                          bool add_prior,
                                          double prior_variance) {
  addNewTempNode(key, initial_pose);

  if (add_prior) {
    addPrior(key, initial_pose, prior_variance, true);
  }
}

void DeformationGraph::addNewTempNode(const gtsam::Key& key,
                                      const gtsam::Pose3& initial_pose) {
  temp_pg_initial_poses_[key] = initial_pose;
  temp_info_.values.insert(key, initial_pose);
}

void DeformationGraph::processNewTempNodesValences(const NodeValenceInfoList& info,
                                                   bool add_prior,
                                                   double edge_variance,
                                                   double prior_variance) {
  for (const auto& factor : info) {
    // Note(Yun) here we do not directly call process new temp and valences
    // To avoid calling process in a process and causing a deadlock
    addNewTempNode(factor.key, factor.pose);

    if (add_prior) {
      // Add temp prior
      addPrior(factor.key, factor.pose, prior_variance, true);
    }

    // Add the consistency factors
    for (Vertex v : factor.valence) {
      const gtsam::Symbol vertex(factor.valence_prefix, v);

      gtsam::Pose3 node_pose;
      gtsam::Point3 vertex_pos;
      if (!checkNodeValence(factor.key, vertex, node_pose, vertex_pos)) {
        continue;
      }

      addDeformationEdge(
          factor.key, vertex, node_pose, vertex_pos, edge_variance, true, false);
      addDeformationEdge(vertex,
                         factor.key,
                         gtsam::Pose3(gtsam::Rot3(), vertex_pos),
                         node_pose.translation(),
                         edge_variance,
                         true,
                         false);
    }
  }
}

void DeformationGraph::removePriorsWithPrefix(char prefix) {
  // First make copy of info_.factors
  const auto nfg_copy = info_.factors;
  const auto temp_nfg_copy = temp_info_.factors;
  // Clear info_.factors
  info_.factors.resize(0);
  temp_info_.factors.resize(0);

  // Iterate and pick out non prior factors and prior factors without key with
  // prefix
  for (auto factor : nfg_copy) {
    const auto prior_factor_3d = dynamic_cast<const PosePrior*>(factor.get());
    if (prior_factor_3d) {
      gtsam::Symbol node(prior_factor_3d->key());
      if (node.chr() != prefix) {
        info_.factors.add(factor);
      }
      continue;
    }

    info_.factors.add(factor);
  }

  for (auto temp_factor : temp_nfg_copy) {
    const auto prior_factor_3d = dynamic_cast<const PosePrior*>(temp_factor.get());
    if (prior_factor_3d) {
      gtsam::Symbol node(prior_factor_3d->key());
      if (node.chr() != prefix) {
        info_.factors.add(temp_factor);
      }
      continue;
    }

    temp_info_.factors.add(temp_factor);
  }
}

pcl::PolygonMesh DeformationGraph::deformMesh(const pcl::PolygonMesh& original_mesh,
                                              const std::vector<Timestamp>& stamps,
                                              const std::vector<int>& graph_indices,
                                              char prefix,
                                              size_t k,
                                              double tol_t) {
  return deformMesh(
      original_mesh, stamps, graph_indices, prefix, info_.values, k, tol_t);
}

pcl::PolygonMesh DeformationGraph::deformMesh(const pcl::PolygonMesh& original_mesh,
                                              const std::vector<Timestamp>& stamps,
                                              const std::vector<int>& graph_indices,
                                              char prefix,
                                              const gtsam::Values& optimized_values,
                                              size_t k,
                                              double tol_t) {
  // extract original vertices
  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  pcl::fromPCLPointCloud2(original_mesh.cloud, new_vertices);

  // note that there's no aliasing w.r.t. the new / old vertices so this is relatively
  // safe
  deformPoints(new_vertices,
               new_vertices,
               stamps,
               prefix,
               optimized_values,
               k,
               tol_t,
               &graph_indices);

  // With new vertices, construct new polygon mesh
  pcl::PolygonMesh new_mesh;
  new_mesh.polygons = original_mesh.polygons;
  pcl::toPCLPointCloud2(new_vertices, new_mesh.cloud);
  return new_mesh;
}

void DeformationGraph::deformPoints(
    pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const pcl::PointCloud<pcl::PointXYZRGBA>& old_vertices,
    const std::vector<Timestamp>& stamps,
    char prefix,
    const gtsam::Values& optimized_values,
    size_t k,
    double tol_t,
    const std::vector<int>* graph_indices,
    int start_index_hint,
    std::vector<std::set<size_t>>* vertex_graph_map) {
  deformPoints(vertices,
               ConstStampedCloud<pcl::PointXYZRGBA>{old_vertices, stamps},
               prefix,
               optimized_values,
               k,
               tol_t,
               graph_indices,
               start_index_hint,
               vertex_graph_map);
}

std::vector<gtsam::Pose3> DeformationGraph::getTrajectory(char prefix) const {
  // return the optimized trajectory (pose graph)
  std::vector<gtsam::Pose3> traj;

  for (size_t i = 0; i < pg_initial_poses_.at(prefix).size(); i++) {
    gtsam::Symbol node(prefix, i);
    if (!info_.values.exists(node)) {
      break;
    }
    traj.push_back(info_.values.at<gtsam::Pose3>(node));
  }
  return traj;
}

std::optional<uint64_t> maybeGetTimestamp(const RobotTimestampMap& timestamps,
                                          int32_t robot,
                                          uint64_t key) {
  auto riter = timestamps.find(robot);
  if (riter == timestamps.end()) {
    return std::nullopt;
  }

  if (key >= riter->second.size()) {
    return std::nullopt;
  }

  return riter->second[key];
}

bool DeformationGraph::tryConvertFactorToPriorEdge(gtsam::NonlinearFactor* factor,
                                                   const RobotTimestampMap& timestamps,
                                                   int factor_idx,
                                                   PoseGraphEdge& edge) const {
  // check if prior factor
  const auto factor_ptr = dynamic_cast<const PosePrior*>(factor);
  if (!factor_ptr) {
    return false;
  }

  const gtsam::Symbol key(factor_ptr->key());
  if (!robot_prefix_to_id.count(key.chr())) {
    return false;
  }

  edge.key_from = key.index();
  edge.key_to = key.index();
  edge.robot_from = robot_prefix_to_id.at(key.chr());
  edge.robot_to = robot_prefix_to_id.at(key.chr());
  const auto stamp_ns = maybeGetTimestamp(timestamps, edge.robot_to, edge.key_to);
  if (stamp_ns) {
    edge.stamp_ns = *stamp_ns;
  }

  edge.pose = factor_ptr->prior().matrix();
  if (factor_idx >= 0 && info_.inlier_weights.size() > factor_idx &&
      info_.inlier_weights.at(factor_idx) < 0.5) {
    edge.type = EdgeType::REJECTED_PRIOR;
  } else {
    edge.type = EdgeType::PRIOR;
  }

  const auto noise_model =
      dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor_ptr->noiseModel().get());
  if (noise_model) {
    edge.covariance = noise_model->covariance();
  }
  return true;
}

bool DeformationGraph::tryConvertFactorToBetweenEdge(
    gtsam::NonlinearFactor* factor,
    const RobotTimestampMap& timestamps,
    int factor_idx,
    PoseGraphEdge& edge) const {
  // check if between factor
  const auto factor_ptr = dynamic_cast<const PoseBetween*>(factor);
  if (!factor_ptr) {
    return false;
  }

  const gtsam::Symbol front(factor_ptr->front());
  const gtsam::Symbol back(factor_ptr->back());
  edge.key_from = front.index();
  edge.key_to = back.index();
  edge.robot_from = robot_prefix_to_id.at(front.chr());
  edge.robot_to = robot_prefix_to_id.at(back.chr());
  const auto stamp_ns = maybeGetTimestamp(timestamps, edge.robot_to, edge.key_to);
  if (stamp_ns) {
    edge.stamp_ns = *stamp_ns;
  }

  bool same_robot = edge.robot_from == edge.robot_to;
  if (same_robot && edge.key_to == edge.key_from + 1) {
    edge.type = EdgeType::ODOM;
  } else {
    if (factor_idx >= 0 && info_.inlier_weights.size() > factor_idx &&
        info_.inlier_weights.at(factor_idx) < 0.5) {
      edge.type = EdgeType::REJECTED_LOOPCLOSE;
    } else {
      edge.type = EdgeType::LOOPCLOSE;
    }
  }

  edge.pose = factor_ptr->measured().matrix();

  const auto noise_model =
      dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor_ptr->noiseModel().get());
  if (noise_model) {
    edge.covariance = noise_model->covariance();
  }
  return true;
}

bool DeformationGraph::tryConvertFactorToDeformationEdge(gtsam::NonlinearFactor* factor,
                                                         PoseGraphEdge& edge) const {
  // check if deformation edge factor
  const auto factor_ptr = dynamic_cast<const DeformationEdgeFactor*>(factor);
  if (!factor_ptr) {
    return false;
  }

  const gtsam::Symbol front(factor_ptr->front());
  const gtsam::Symbol back(factor_ptr->back());
  edge.key_from = front.index();
  edge.key_to = back.index();

  if (vertex_prefix_to_id.count(front.chr()) && vertex_prefix_to_id.count(back.chr())) {
    edge.type = EdgeType::MESH;
    edge.robot_from = vertex_prefix_to_id.at(front.chr());
    edge.robot_to = vertex_prefix_to_id.at(back.chr());
    edge.stamp_ns = vertices_.at(front.chr()).at(front.index()).timestamp_ns;
  } else if (vertex_prefix_to_id.count(front.chr())) {
    edge.type = EdgeType::MESH_POSE;
    edge.robot_from = vertex_prefix_to_id.at(front.chr());
    edge.robot_to = robot_prefix_to_id.at(back.chr());
    edge.stamp_ns = vertices_.at(front.chr()).at(front.index()).timestamp_ns;
  } else if (vertex_prefix_to_id.count(back.chr())) {
    edge.type = EdgeType::POSE_MESH;
    edge.robot_from = robot_prefix_to_id.at(front.chr());
    edge.robot_to = vertex_prefix_to_id.at(back.chr());
    edge.stamp_ns = vertices_.at(back.chr()).at(back.index()).timestamp_ns;
  } else {
    return false;
  }
  edge.pose = gtsam::Pose3(gtsam::Rot3(), factor_ptr->measurement()).matrix();

  const auto noise_model =
      dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor_ptr->noiseModel().get());
  if (noise_model) {
    edge.covariance.block(3, 3, 3, 3) = noise_model->covariance();
  }

  return true;
}

bool DeformationGraph::tryConvertKeyToPoseNode(const gtsam::Key& key,
                                               const RobotTimestampMap& timestamps,
                                               PoseGraphNode& node,
                                               bool optimized) const {
  gtsam::Symbol node_symb(key);
  if (!robot_prefix_to_id.count(node_symb.chr())) {
    return false;
  }

  const size_t robot_id = robot_prefix_to_id.at(node_symb.chr());
  node.key = node_symb.index();
  node.robot_id = robot_id;
  const auto stamp_ns = maybeGetTimestamp(timestamps, node.robot_id, node.key);
  if (stamp_ns) {
    node.stamp_ns = *stamp_ns;
  } else {
    SPARK_LOG(WARNING) << "Invalid timestamp for (robot=" << node.robot_id
                       << ", pose=" << node.key << ")!";
  }
  node.pose =
      optimized ? info_.values.at<gtsam::Pose3>(key).matrix()
                : pg_initial_poses_.at(node_symb.chr()).at(node_symb.index()).matrix();
  return true;
}

size_t DeformationGraph::getRemappedId(const std::map<size_t, size_t>& remap,
                                       size_t original) const {
  const auto iter = remap.find(original);
  return iter == remap.end() ? original : iter->second;
}

bool DeformationGraph::checkAdjacency(gtsam::Key from, gtsam::Key to) const {
  if (!adjacency_map_.count(from)) {
    return false;
  }

  if (!adjacency_map_.at(from).count(to)) {
    return false;
  }
  return true;
}

bool DeformationGraph::tryConvertKeyToMeshNode(const gtsam::Key& key,
                                               PoseGraphNode& node,
                                               bool optimized) const {
  gtsam::Symbol node_symb(key);

  if (!vertex_prefix_to_id.count(node_symb.chr())) {
    return false;
  }

  const size_t robot_id = vertex_prefix_to_id.at(node_symb.chr());
  node.key = node_symb.index();
  node.robot_id = robot_id;
  node.stamp_ns = vertices_.at(node_symb.chr()).at(node_symb.index()).timestamp_ns;
  node.pose =
      optimized
          ? info_.values.at<gtsam::Pose3>(key).matrix()
          : gtsam::Pose3(gtsam::Rot3(),
                         vertices_.at(node_symb.chr()).at(node_symb.index()).position)
                .matrix();
  return true;
}

PoseGraph::Ptr DeformationGraph::getPoseGraph(const RobotTimestampMap& timestamps,
                                              bool include_deformation_edges,
                                              bool include_between_edges,
                                              bool optimized) const {
  auto graph = std::make_shared<PoseGraph>();

  // first store the factors as edges
  size_t index = 0;
  for (const auto& factor : info_.factors) {
    PoseGraphEdge pg_edge;
    if (include_between_edges &&
        tryConvertFactorToBetweenEdge(factor.get(), timestamps, index, pg_edge)) {
      graph->edges.push_back(pg_edge);
    }

    if (include_between_edges &&
        tryConvertFactorToPriorEdge(factor.get(), timestamps, index, pg_edge)) {
      graph->edges.push_back(pg_edge);
    }

    if (include_deformation_edges &&
        tryConvertFactorToDeformationEdge(factor.get(), pg_edge)) {
      graph->edges.push_back(pg_edge);
    }
    index++;
  }

  // Then store the values as nodes
  for (const auto& key : info_.values.keys()) {
    PoseGraphNode pg_node;
    if (include_between_edges &&
        tryConvertKeyToPoseNode(key, timestamps, pg_node, optimized)) {
      graph->nodes.push_back(pg_node);
    }

    if (include_deformation_edges && tryConvertKeyToMeshNode(key, pg_node, optimized)) {
      graph->nodes.push_back(pg_node);
    }
  }

  return graph;
}

void DeformationGraph::updateValues(const gtsam::Values& updates) {
  info_.values.update(updates);
}

void DeformationGraph::updateTempValues(const gtsam::Values& updates) {
  temp_info_.values.update(updates);
}

void DeformationGraph::updateInlierWeights(const std::vector<double>& weights) {
  info_.inlier_weights = weights;
}

void DeformationGraph::updateTempInlierWeights(const std::vector<double>& weights) {
  temp_info_.inlier_weights = weights;
}

std::vector<size_t> DeformationGraph::getClosestVertexIndices(
    size_t robot_id, Timestamp stamp_ns) const {
  auto iter = vertices_.find(GetVertexPrefix(robot_id));
  if (iter == vertices_.end() || iter->second.empty()) {
    return {};
  }

  // TODO(Yun): Implicit assumption here that the vertices are ordered (normally are
  // since that's how processIncrementalMeshGraph works) but might need to account for
  // out-of-order timestamps
  const auto lower = std::lower_bound(iter->second.begin(),
                                      iter->second.end(),
                                      stamp_ns,
                                      [](const DeformationVertex& v, Timestamp stamp) {
                                        return v.timestamp_ns < stamp;
                                      });
  const auto idx = std::distance(iter->second.begin(), lower);

  Timestamp closest;
  if (idx == 0) {
    closest = iter->second.front().timestamp_ns;
  } else if (idx == iter->second.size()) {
    closest = iter->second.back().timestamp_ns;
  } else {
    const auto prev = iter->second.at(idx - 1).timestamp_ns;
    const auto curr = iter->second.at(idx).timestamp_ns;
    closest = (stamp_ns - prev <= curr - stamp_ns) ? prev : curr;
  }

  // TODO(nathan) use ordered vertices to avoid iterating over whole set of vertices
  // Collect indices with same timestamp
  std::vector<size_t> matches;
  for (size_t i = 0; i < iter->second.size(); i++) {
    if (iter->second[i].timestamp_ns == closest) {
      matches.push_back(i);
    }
  }

  return matches;
}

}  // namespace kimera_pgmo
