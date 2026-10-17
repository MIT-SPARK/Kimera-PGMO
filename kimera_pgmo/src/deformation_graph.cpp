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

#include <cmath>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph_4dof.h"
#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;

using ControlPointMap = std::map<char, std::vector<StampedPoint>>;
using TrajectoryMap = std::map<char, std::vector<StampedPose>>;

using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;
using PosePrior = gtsam::PriorFactor<gtsam::Pose3>;
using EdgeType = pose_graph_tools::PoseGraphEdge::Type;

namespace {

template <typename Predicate>
void retainFactors(gtsam::NonlinearFactorGraph& factors,
                   std::set<size_t>& inliers,
                   std::vector<double>& weights,
                   const Predicate& keep) {
  gtsam::NonlinearFactorGraph retained;
  std::set<size_t> retained_inliers;
  std::vector<double> retained_weights;
  for (size_t i = 0; i < factors.size(); ++i) {
    const auto& factor = factors[i];
    if (!factor || !keep(*factor)) {
      continue;
    }

    if (inliers.count(i)) {
      retained_inliers.insert(retained.size());
    }

    if (i < weights.size()) {
      retained_weights.push_back(weights[i]);
    }

    retained.push_back(factor);
  }

  factors = std::move(retained);
  inliers = std::move(retained_inliers);
  weights = std::move(retained_weights);
}

template <typename FactorT>
const gtsam::noiseModel::Gaussian* factorNoise(const FactorT& factor) {
  return dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor.noiseModel().get());
}

inline std::optional<uint64_t> maybeGetTimestamp(const TrajectoryMap& trajectories,
                                                 gtsam::Symbol key) {
  auto riter = trajectories.find(key.chr());
  if (riter == trajectories.end()) {
    return std::nullopt;
  }

  if (key.index() >= riter->second.size()) {
    return std::nullopt;
  }

  return riter->second[key.index()].timestamp_ns;
}

inline bool isOutlier(const std::vector<double>& inlier_weights, int factor_idx) {
  if (inlier_weights.empty()) {
    return false;
  }

  if (factor_idx < 0 || static_cast<size_t>(factor_idx) >= inlier_weights.size()) {
    return false;
  }

  return inlier_weights.at(factor_idx) < 0.5;
}

inline bool factorToDeformationEdge(const gtsam::NonlinearFactor* factor,
                                    const ControlPointMap& control_points,
                                    PoseGraphEdge& edge) {
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
    edge.stamp_ns = control_points.at(front.chr()).at(front.index()).timestamp_ns;
  } else if (vertex_prefix_to_id.count(front.chr())) {
    edge.type = EdgeType::MESH_POSE;
    edge.robot_from = vertex_prefix_to_id.at(front.chr());
    edge.robot_to = robot_prefix_to_id.at(back.chr());
    edge.stamp_ns = control_points.at(front.chr()).at(front.index()).timestamp_ns;
  } else if (vertex_prefix_to_id.count(back.chr())) {
    edge.type = EdgeType::POSE_MESH;
    edge.robot_from = robot_prefix_to_id.at(front.chr());
    edge.robot_to = vertex_prefix_to_id.at(back.chr());
    edge.stamp_ns = control_points.at(back.chr()).at(back.index()).timestamp_ns;
  } else {
    return false;
  }

  edge.pose = gtsam::Pose3(gtsam::Rot3(), factor_ptr->measurement()).matrix();
  const auto noise_model = factorNoise(*factor_ptr);
  if (noise_model) {
    edge.covariance.block(3, 3, 3, 3) = noise_model->covariance();
  }

  return true;
}

inline bool factorToPrior(const gtsam::NonlinearFactor* factor,
                          const TrajectoryMap& trajectories,
                          const std::vector<double>& inlier_weights,
                          int factor_idx,
                          PoseGraphEdge& edge) {
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
  const auto stamp_ns = maybeGetTimestamp(trajectories, key);
  if (stamp_ns) {
    edge.stamp_ns = *stamp_ns;
  }

  edge.type = isOutlier(inlier_weights, factor_idx) ? EdgeType::REJECTED_PRIOR
                                                    : EdgeType::PRIOR;
  edge.pose = factor_ptr->prior().matrix();
  const auto noise_model = factorNoise(*factor_ptr);
  if (noise_model) {
    edge.covariance = noise_model->covariance();
  }

  return true;
}

inline bool factorToBetween(const gtsam::NonlinearFactor* factor,
                            const TrajectoryMap& trajectories,
                            const std::vector<double>& inlier_weights,
                            int factor_idx,
                            PoseGraphEdge& edge) {
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
  const auto stamp_ns = maybeGetTimestamp(trajectories, back);
  if (stamp_ns) {
    edge.stamp_ns = *stamp_ns;
  }

  bool same_robot = edge.robot_from == edge.robot_to;
  if (same_robot && edge.key_to == edge.key_from + 1) {
    edge.type = EdgeType::ODOM;
  } else {
    edge.type = isOutlier(inlier_weights, factor_idx) ? EdgeType::REJECTED_LOOPCLOSE
                                                      : EdgeType::LOOPCLOSE;
  }

  edge.pose = factor_ptr->measured().matrix();
  const auto noise_model = factorNoise(*factor_ptr);
  if (noise_model) {
    edge.covariance = noise_model->covariance();
  }

  return true;
}

inline bool valueToPoseNode(const gtsam::Key& key,
                            const gtsam::Values& values,
                            const TrajectoryMap& trajectories,
                            PoseGraphNode& node,
                            bool optimized) {
  const gtsam::Symbol node_symb(key);
  if (!robot_prefix_to_id.count(node_symb.chr())) {
    return false;
  }

  const size_t robot_id = robot_prefix_to_id.at(node_symb.chr());
  const auto init_pose = trajectories.at(node_symb.chr()).at(node_symb.index());

  node.key = node_symb.index();
  node.robot_id = robot_id;
  node.stamp_ns = init_pose.timestamp_ns;
  if (optimized) {
    node.pose = values.at<gtsam::Pose3>(key).matrix();
  } else {
    node.pose = init_pose.pose.matrix();
  }

  return true;
}

inline bool valueToMeshNode(const gtsam::Key& key,
                            const gtsam::Values& values,
                            const ControlPointMap& control_points,
                            PoseGraphNode& node,
                            bool optimized) {
  const gtsam::Symbol node_symb(key);
  if (!vertex_prefix_to_id.count(node_symb.chr())) {
    return false;
  }

  const size_t robot_id = vertex_prefix_to_id.at(node_symb.chr());
  const auto point = control_points.at(node_symb.chr()).at(node_symb.index());

  node.key = node_symb.index();
  node.robot_id = robot_id;
  node.stamp_ns = point.timestamp_ns;
  if (optimized) {
    node.pose = values.at<gtsam::Pose3>(key).matrix();
  } else {
    node.pose = gtsam::Pose3(gtsam::Rot3(), point.pos).matrix();
  }

  return true;
}

}  // namespace

DeformationGraph::DeformationGraph(bool add_init_vertex_prior)
    : add_init_vertex_prior_(add_init_vertex_prior), num_loopclosures_(0) {}

DeformationGraph::~DeformationGraph() {}

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

    // Note that the pose here directly updates the pg initial pose. Which matters for
    // the processNodeValence function (connecting node to vertex) but does not matter
    // as much if the node to vertex edges will be directly given
    addNewNode(node_key, node.stamp_ns, node_pose, false);
  }

  for (const auto& edge : pose_graph.edges) {
    if (!variance_map.count(edge.type)) {
      SPARK_LOG(ERROR) << "Missing variance for edge '" << edge.type
                       << "' in processPoseGraph";
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
    addDeformationEdge(
        from_key, to_key, from_T_to.translation(), variance_map.at(edge.type), true);
  }
}

void DeformationGraph::processNodeValence(gtsam::Key key,
                                          const std::vector<uint64_t>& valences,
                                          char valence_prefix,
                                          double variance) {
  // Add the consistency factors
  for (const auto& v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);

    gtsam::Pose3 node_pose;
    gtsam::Point3 vertex_pos;
    if (!checkNodeValence(key, vertex, node_pose, vertex_pos)) {
      continue;
    }

    addDeformationEdge(key, vertex, node_pose, vertex_pos, variance, true);
    addDeformationEdge(vertex,
                       key,
                       gtsam::Pose3(gtsam::Rot3(), vertex_pos),
                       node_pose.translation(),
                       variance,
                       true);
  }
}

bool DeformationGraph::checkNodeValence(gtsam::Key key,
                                        gtsam::Key vertex,
                                        gtsam::Pose3& node_pose,
                                        gtsam::Point3& vertex_pos) const {
  const auto prefix = gtsam::Symbol(key).chr();
  const auto idx = gtsam::Symbol(key).index();
  const auto valence_prefix = gtsam::Symbol(vertex).chr();
  const auto valence_idx = gtsam::Symbol(vertex).index();
  if (!values_.exists(vertex)) {
    return false;
  }

  node_pose = trajectories_.at(prefix).at(idx).pose;
  vertex_pos = control_points_.at(valence_prefix).at(valence_idx).pos;
  return true;
}

void DeformationGraph::processBetweenAsMeshConnections(const gtsam::Pose3& w_T_src,
                                                       const std::vector<uint64_t>& src,
                                                       const gtsam::Pose3& w_T_dst,
                                                       const std::vector<uint64_t>& dst,
                                                       const gtsam::Pose3& src_T_dst,
                                                       char src_prefix,
                                                       char dst_prefix,
                                                       double variance,
                                                       bool known_inliers) {
  for (const auto& s : src) {
    const gtsam::Symbol vertex_s(src_prefix, s);
    auto w_T_s = gtsam::Pose3(gtsam::Rot3(), control_points_[src_prefix].at(s).pos);
    auto s_T_source = w_T_s.between(w_T_src);
    for (const auto& d : dst) {
      const gtsam::Symbol vertex_d(dst_prefix, d);
      if (vertex_s == vertex_d) {
        continue;  // Do not add edge to self
      }

      // For now we do not duplicate deformation graph edges even if there are multiple
      // between factors connecting them. Hence the adjacency book-keeping.
      if (adjacency_map_.count(vertex_s) &&
          adjacency_map_.at(vertex_s).count(vertex_d)) {
        continue;
      }

      auto w_T_d = gtsam::Pose3(gtsam::Rot3(), control_points_[dst_prefix].at(d).pos);
      auto d_T_dest = w_T_d.between(w_T_dst);
      auto s_T_d = s_T_source.compose(src_T_dst).compose(d_T_dest.inverse());
      auto d_T_s = s_T_d.inverse();
      addDeformationEdge(
          vertex_s, vertex_d, s_T_d.translation(), variance, known_inliers);
      addDeformationEdge(
          vertex_d, vertex_s, d_T_s.translation(), variance, known_inliers);
    }
  }
}

void DeformationGraph::processPointMeasurement(gtsam::Key from_key,
                                               gtsam::Key to_key,
                                               const gtsam::Pose3& from_pose,
                                               const gtsam::Point3& to_point,
                                               double variance,
                                               bool known_inlier) {
  if (!values_.exists(to_key)) {
    values_.insert(to_key, gtsam::Pose3(gtsam::Rot3(), to_point));
  }

  addDeformationEdge(from_key, to_key, from_pose, to_point, variance, known_inlier);
}

void DeformationGraph::addDeformationEdge(const gtsam::Key& from_key,
                                          const gtsam::Key& to_key,
                                          const gtsam::Pose3& from_pose,
                                          const gtsam::Point3& to_point,
                                          double variance,
                                          bool known_inlier) {
  const auto noise = gtsam::noiseModel::Isotropic::Variance(3, variance);
  const DeformationEdgeFactor new_edge(from_key, to_key, from_pose, to_point, noise);

  auto& adjacent_keys = adjacency_map_[from_key];
  adjacent_keys.insert(to_key);
  if (known_inlier) {
    known_inliers_.insert(nfg_.size());
  }

  nfg_.add(new_edge);
}

void DeformationGraph::addDeformationEdge(const gtsam::Key& from_key,
                                          const gtsam::Key& to_key,
                                          const gtsam::Point3& measurement,
                                          double variance,
                                          bool known_inlier) {
  const auto noise = gtsam::noiseModel::Isotropic::Variance(3, variance);
  const DeformationEdgeFactor new_edge(from_key, to_key, measurement, noise);

  auto& adjacent_keys = adjacency_map_[from_key];
  adjacent_keys.insert(to_key);
  if (known_inlier) {
    known_inliers_.insert(nfg_.size());
  }

  nfg_.add(new_edge);
}

void DeformationGraph::addPrior(const gtsam::Key& key,
                                const gtsam::Pose3& pose,
                                double variance,
                                bool known_inlier) {
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  const auto noise = gtsam::noiseModel::Diagonal::Variances(variances);

  gtsam::PriorFactor<gtsam::Pose3> measurement(key, pose, noise);
  if (known_inlier) {
    known_inliers_.insert(nfg_.size());
  }

  nfg_.add(measurement);
}

void DeformationGraph::processNodeMeasurements(const NodeMeasurements& measurements,
                                               double variance) {
  for (const auto& [key, pose] : measurements) {
    if (!values_.exists(key)) {
      SPARK_LOG(ERROR) << "DeformationGraph: adding node measurement to a node "
                       << gtsam::DefaultKeyFormatter(key) << " not seen before.";
      values_.insert(key, pose);
    } else {
      values_.update(key, pose);
    }

    addPrior(key, pose, variance);
  }
}

void DeformationGraph::processNewBetween(gtsam::Key key_from,
                                         gtsam::Key key_to,
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

  // Check if prefixes exist in pg_initial_poses_
  if (!trajectories_.count(from_prefix)) {
    return false;
  }

  if (!trajectories_.count(to_prefix)) {
    return false;
  }

  if (from_idx >= trajectories_.at(from_prefix).size()) {
    SPARK_LOG(ERROR)
        << "DeformationGraph: when adding new between from key should already exist.";
    return false;
  }

  if (to_idx >= trajectories_.at(to_prefix).size()) {
    SPARK_LOG(ERROR) << "DeformationGraph: skipping keys in addNewBetween.";
    return false;
  }

  return true;
}

void DeformationGraph::updatePoseGraphInitialGuess(gtsam::Key key_from,
                                                   gtsam::Key key_to,
                                                   Timestamp stamp,
                                                   const gtsam::Pose3& meas) {
  const auto to_prefix = gtsam::Symbol(key_to).chr();

  gtsam::Pose3 initial_estimate;
  if (values_.exists(key_from)) {
    initial_estimate = values_.at<gtsam::Pose3>(key_from).compose(meas);
  } else {
    SPARK_LOG(FATAL) << "Missing from key values when adding initial guess.";
  }

  auto& trajectory = trajectories_.at(to_prefix);
  values_.insert(key_to, initial_estimate);
  const auto init_pose = trajectory.back().pose.compose(meas);
  trajectory.push_back({stamp, init_pose});
}

void DeformationGraph::addNewBetween(const gtsam::Key& key_from,
                                     const gtsam::Key& key_to,
                                     const gtsam::Pose3& meas,
                                     double variance,
                                     bool known_inlier) {
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  const auto noise = gtsam::noiseModel::Diagonal::Variances(variances);

  if (known_inlier) {
    known_inliers_.insert(nfg_.size());
  }

  nfg_.add(gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  // if it's a loop closure factor
  if (key_to != key_from + 1) {
    SPARK_LOG(DEBUG) << "DeformationGraph: Added loop closure";
    num_loopclosures_++;
  }
}

bool DeformationGraph::addNewMeshNode(const gtsam::Key& node_key,
                                      const gtsam::Pose3& node_pose,
                                      const Timestamp& node_stamp) {
  char node_prefix = gtsam::Symbol(node_key).chr();
  size_t node_idx = gtsam::Symbol(node_key).index();
  auto iter = control_points_.find(node_prefix);
  if (iter == control_points_.end()) {
    iter = control_points_.emplace(node_prefix, std::vector<StampedPoint>{}).first;
  }

  auto& points = iter->second;
  if (node_idx != points.size()) {
    return false;
  }

  points.push_back({node_stamp, node_pose.translation()});
  if (add_init_vertex_prior_ && values_.size() == 0) {
    // TODO(Yun) temporary hack, check if this assumption always valid even with poses
    addPrior(node_key, node_pose, 1e-3, true);
  }

  values_.insert(node_key, node_pose);
  return true;
}

bool DeformationGraph::checkNewMeshNode(const gtsam::Key& node_key) const {
  char node_prefix = gtsam::Symbol(node_key).chr();
  size_t node_idx = gtsam::Symbol(node_key).index();
  auto iter = control_points_.find(node_prefix);
  if (iter == control_points_.end()) {
    // initial node won't have created control points for prefix
    return node_idx == 0;
  }

  // The check here returns true even for duplicated (added) nodes
  // Only return false if there is a likely message drop
  return node_idx <= iter->second.size();
}

bool DeformationGraph::checkNewMeshEdge(const gtsam::Key& from,
                                        const gtsam::Key& to) const {
  const gtsam::Symbol from_symb(from);
  const gtsam::Symbol to_symb(to);
  if (from_symb.index() >= control_points_.at(from_symb.chr()).size() ||
      to_symb.index() >= control_points_.at(to_symb.chr()).size()) {
    return false;
  }

  if (!values_.exists(from) || !values_.exists(to)) {
    return false;
  }

  return true;
}

void DeformationGraph::processNewMeshEdgesAndNodes(const MeshEdges& mesh_edges,
                                                   const gtsam::Values& mesh_nodes,
                                                   const MeshNodeStamps& node_stamps,
                                                   std::vector<size_t>* added_indices,
                                                   std::vector<Timestamp>* added_stamps,
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
      added_stamps->push_back(node_stamps.at(node_key));
    }
  }

  // Iterate and add the new edges
  for (auto e : mesh_edges) {
    if (!checkNewMeshEdge(e.first, e.second)) {
      SPARK_LOG(FATAL) << "Error adding new mesh edge.";
    }

    const auto pose_from = mesh_nodes.at<gtsam::Pose3>(e.first);
    const auto point_to = mesh_nodes.at<gtsam::Pose3>(e.second).translation();
    addDeformationEdge(e.first, e.second, pose_from, point_to, variance, true);
  }
}

bool DeformationGraph::addNewNode(gtsam::Key key,
                                  Timestamp stamp,
                                  const gtsam::Pose3& initial_pose,
                                  bool add_prior,
                                  double prior_variance) {
  const char prefix = gtsam::Symbol(key).chr();
  const size_t idx = gtsam::Symbol(key).index();

  // NOTE(nathan) this is not ideal, but will get refactored at some point
  auto iter = trajectories_.find(prefix);
  if (iter == trajectories_.end()) {
    iter = trajectories_.emplace(prefix, std::vector<StampedPose>()).first;
  }

  if (idx > iter->second.size()) {
    SPARK_LOG(ERROR) << "DeformationGraph: Nodes skipped in pose graph nodes";
    return false;
  }

  if (idx < iter->second.size()) {
    return false;  // Duplicate node
  }

  values_.insert(key, initial_pose);
  iter->second.push_back({stamp, initial_pose});
  if (add_prior) {
    addPrior(key, initial_pose, prior_variance);
  }

  return true;
}

void DeformationGraph::removePriorsWithPrefix(char prefix) {
  retainFactors(nfg_, known_inliers_, inlier_weights_, [prefix](const auto& factor) {
    const auto prior = dynamic_cast<const PosePrior*>(&factor);
    return !prior || gtsam::Symbol(prior->key()).chr() != prefix;
  });
}

size_t DeformationGraph::getNumVertices() const {
  size_t num_vertices = 0;
  for (const auto& [prefix, vertices] : control_points_) {
    num_vertices += vertices.size();
  }

  return num_vertices;
}

std::vector<gtsam::Pose3> DeformationGraph::getTrajectory(char prefix) const {
  std::vector<gtsam::Pose3> traj;
  for (size_t i = 0; i < trajectories_.at(prefix).size(); i++) {
    gtsam::Symbol node(prefix, i);
    if (!values_.exists(node)) {
      break;
    }

    traj.push_back(values_.at<gtsam::Pose3>(node));
  }

  return traj;
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

PoseGraph::Ptr DeformationGraph::getPoseGraph(bool include_deformation_edges,
                                              bool include_between_edges,
                                              bool optimized) const {
  auto graph = std::make_shared<PoseGraph>();

  // first store the factors as edges
  size_t idx = 0;
  for (const auto& factor : nfg_) {
    PoseGraphEdge edge;
    if (include_between_edges &&
        factorToBetween(factor.get(), trajectories_, inlier_weights_, idx, edge)) {
      graph->edges.push_back(edge);
    }

    if (include_between_edges &&
        factorToPrior(factor.get(), trajectories_, inlier_weights_, idx, edge)) {
      graph->edges.push_back(edge);
    }

    if (include_deformation_edges &&
        factorToDeformationEdge(factor.get(), control_points_, edge)) {
      graph->edges.push_back(edge);
    }

    ++idx;
  }

  // Then store the values as nodes
  for (const auto& key : values_.keys()) {
    PoseGraphNode node;
    if (include_between_edges &&
        valueToPoseNode(key, values_, trajectories_, node, optimized)) {
      graph->nodes.push_back(node);
    }

    if (include_deformation_edges &&
        valueToMeshNode(key, values_, control_points_, node, optimized)) {
      graph->nodes.push_back(node);
    }
  }

  return graph;
}

void DeformationGraph::updateValues(const gtsam::Values& updates) {
  values_.update(updates);
}

void DeformationGraph::updateInlierWeights(const std::vector<double>& weights) {
  inlier_weights_ = weights;
}

gtsam::Pose3 DeformationGraph::getInitialPose(char prefix, size_t index) const {
  return trajectories_.at(prefix).at(index).pose;
}

gtsam::Point3 DeformationGraph::getInitialPositionVertex(char prefix,
                                                         size_t index) const {
  return control_points_.at(prefix).at(index).pos;
}

bool DeformationGraph::hasVertexKey(char prefix) const {
  return control_points_.count(prefix);
}

bool DeformationGraph::hasPrefixPoses(char prefix) const {
  return trajectories_.count(prefix);
}

std::unique_lock<std::mutex> DeformationGraph::acquireLock() {
  return std::unique_lock<std::mutex>(mutex_);
}

std::vector<size_t> DeformationGraph::findClosestMeshIndices(size_t robot_id,
                                                             Timestamp stamp) const {
  std::vector<size_t> indices;
  const auto prefix = GetVertexPrefix(robot_id);
  auto iter = control_points_.find(prefix);
  if (iter == control_points_.end() || iter->second.empty()) {
    return indices;
  }

  // TODO(Yun): Implicit assumption here that the mesh stamps are ordered (normally are
  // since that's how processIncrementalMeshGraph works) but might need to account for
  // out-of-order timestamps
  const auto& points = iter->second;
  auto lower = std::lower_bound(
      points.begin(), points.end(), stamp, [](const auto& lhs, const auto& rhs) {
        return lhs.timestamp_ns < rhs;
      });
  size_t idx = std::distance(points.begin(), lower);

  Timestamp closest;
  if (idx == 0) {
    closest = points.front().timestamp_ns;
  } else if (idx == points.size()) {
    closest = points.back().timestamp_ns;
  } else {
    const auto prev = points[idx - 1].timestamp_ns;
    const auto curr = points[idx].timestamp_ns;
    closest = (stamp - prev <= curr - stamp) ? prev : curr;
  }

  // Collect other indices with same value
  for (size_t i = 0; i < points.size(); i++) {
    if (points[i].timestamp_ns == closest) {
      indices.push_back(i);
    }
  }

  return indices;
}

void DeformationGraph::clear() {
  trajectories_.clear();
  control_points_.clear();

  nfg_.resize(0);
  known_inliers_.clear();
  values_.clear();
  inlier_weights_.clear();

  adjacency_map_.clear();

  num_loopclosures_ = 0;
}

void DeformationGraph::clearMeshNodesOnly() {
  control_points_.clear();
  adjacency_map_.clear();

  // Remove mesh vertices from values_
  gtsam::Values new_values;
  for (const auto& [key, value] : values_) {
    if (!IsMeshVertex(key)) {
      // Keep non-mesh vertices (robot poses)
      new_values.insert(key, value);
    }
  }

  values_ = new_values;

  retainFactors(nfg_, known_inliers_, inlier_weights_, [](const auto& factor) {
    return std::none_of(factor.keys().begin(), factor.keys().end(), IsMeshVertex);
  });
}

void DeformationGraph::clearFactors() {
  nfg_.resize(0);
  known_inliers_.clear();
  inlier_weights_.clear();
}

OptimizationProblem DeformationGraph::getOptimizationProblem(bool use_4dof) const {
  if (!use_4dof) {
    return {values_, nfg_, known_inliers_};
  }

  auto [factors, initial] = convertGraphToPose4DoF(nfg_, values_);
  return {std::move(initial), std::move(factors), known_inliers_};
}

void DeformationGraph::updateOptimizationResult(const OptimizationResult& result) {
  updateValues(convertPose4DoFToPose3(result.estimates));
  updateInlierWeights(result.weights);
}

}  // namespace kimera_pgmo
