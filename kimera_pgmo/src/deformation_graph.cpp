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

#include "kimera_pgmo/pcl_mesh_traits.h"

using pcl::PolygonMesh;

namespace kimera_pgmo {

using MeasurementVector = std::vector<std::pair<gtsam::Key, gtsam::Pose3>>;
using RobotTimestampMap = std::map<size_t, std::vector<Timestamp>>;
using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;
using PosePrior = gtsam::PriorFactor<gtsam::Pose3>;
using EdgeType = pose_graph_tools::PoseGraphEdge::Type;

DeformationGraph::DeformationGraph()
    : verbose_(true),
      pgo_(nullptr),
      force_recalculate_(true),
      recalculate_vertices_(false) {}
DeformationGraph::~DeformationGraph() {}

bool DeformationGraph::initialize(const KimeraRPGO::RobustSolverParams& params) {
  // Initialize RPGO
  pgo_params_ = params;
  pgo_ =
      std::unique_ptr<KimeraRPGO::RobustSolver>(new KimeraRPGO::RobustSolver(params));
  return true;
}

void DeformationGraph::processPoseGraph(const pose_graph_tools::PoseGraph& pose_graph,
                                        const EdgeTypeVarianceMap& variance_map,
                                        std::map<size_t, size_t> robot_id_remap) {
  for (const auto& node : pose_graph.nodes) {
    auto node_robot_id = getRemappedId(robot_id_remap, node.robot_id);
    auto node_key = gtsam::Symbol(robot_id_to_prefix.at(node_robot_id), node.key);
    gtsam::Pose3 node_pose(node.pose.matrix());
    addNewNode(node_key, node_pose);
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
      addNewBetween(from_key, to_key, from_T_to, variance_map.at(edge.type));
    } else {
      addPrior(from_key, from_T_to, variance_map.at(edge.type));
    }
  }
}

void DeformationGraph::processMeshGraph(const pose_graph_tools::PoseGraph& mesh_graph,
                                        const EdgeTypeVarianceMap& variance_map,
                                        std::map<size_t, size_t> robot_id_remap) {
  for (const auto& node : mesh_graph.nodes) {
    auto node_robot_id = getRemappedId(robot_id_remap, node.robot_id);
    auto node_key =
        gtsam::Symbol(robot_id_to_vertex_prefix.at(node_robot_id), node.key);
    gtsam::Pose3 node_pose(node.pose.matrix());
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
        from_key, to_key, from_T_to.translation(), variance_map.at(edge.type));
  }
}

void DeformationGraph::processNodeValence(const gtsam::Key& key,
                                          const Vertices& valences,
                                          const char& valence_prefix,
                                          double variance,
                                          bool temp) {
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  // Add the consistency factors
  for (Vertex v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);
    if (!values_.exists(vertex) && !new_values_.exists(vertex) &&
        !new_temp_values_.exists(vertex)) {
      continue;
    }
    bool non_temp_node =
        pg_initial_poses_.count(prefix) && pg_initial_poses_[prefix].size() > idx;
    const auto node_pose = non_temp_node ? pg_initial_poses_[prefix].at(idx)
                                         : temp_pg_initial_poses_.at(key);
    const gtsam::Pose3 vertex_pose(gtsam::Rot3(),
                                   vertex_positions_[valence_prefix].at(v));

    addDeformationEdge(
        key, vertex, node_pose, vertex_pose.translation(), variance, temp);
    addDeformationEdge(
        vertex, key, vertex_pose, node_pose.translation(), variance, temp);
  }
}

void DeformationGraph::processPointMeasurement(const gtsam::Key& from_key,
                                               const gtsam::Key& to_key,
                                               const gtsam::Pose3& from_pose,
                                               const gtsam::Point3& to_point,
                                               double variance,
                                               bool temp) {
  if (!values_.exists(to_key) && !new_values_.exists(to_key) &&
      !new_temp_values_.exists(to_key)) {
    if (temp) {
      new_temp_values_.insert(to_key, gtsam::Pose3(gtsam::Rot3(), to_point));
    } else {
      new_values_.insert(to_key, gtsam::Pose3(gtsam::Rot3(), to_point));
    }
  }

  addDeformationEdge(from_key, to_key, from_pose, to_point, variance, temp);
}

void DeformationGraph::addDeformationEdge(const gtsam::Key& from_key,
                                          const gtsam::Key& to_key,
                                          const gtsam::Pose3& from_pose,
                                          const gtsam::Point3& to_point,
                                          double variance,
                                          bool temp) {
  // Define noise. Hardcoded for now
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, variance);
  // Create deformation edge factor
  const DeformationEdgeFactor new_edge(from_key, to_key, from_pose, to_point, noise);
  if (temp) {
    new_temp_factors_.add(new_edge);
    return;
  }
  consistency_factors_.add(new_edge);
  new_factors_.add(new_edge);
}

void DeformationGraph::addDeformationEdge(const gtsam::Key& from_key,
                                          const gtsam::Key& to_key,
                                          const gtsam::Point3& measurement,
                                          double variance,
                                          bool temp) {
  // Define noise. Hardcoded for now
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, variance);
  // Create deformation edge factor
  const DeformationEdgeFactor new_edge(from_key, to_key, measurement, noise);
  if (temp) {
    new_temp_factors_.add(new_edge);
    return;
  }
  consistency_factors_.add(new_edge);
  new_factors_.add(new_edge);
}

void DeformationGraph::addPrior(const gtsam::Key& key,
                                const gtsam::Pose3& pose,
                                double variance,
                                bool temp) {
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  const auto noise = gtsam::noiseModel::Diagonal::Variances(variances);

  gtsam::PriorFactor<gtsam::Pose3> measurement(key, pose, noise);
  if (temp) {
    new_temp_factors_.add(measurement);
  } else {
    new_factors_.add(measurement);
  }
}

void DeformationGraph::processNodeMeasurements(const MeasurementVector& measurements,
                                               double variance) {
  for (auto&& [key, pose] : measurements) {
    if (!values_.exists(key)) {
      if (!new_values_.exists(key)) {
        SPARK_LOG(ERROR) << "DeformationGraph: adding node measurement to a node "
                         << gtsam::DefaultKeyFormatter(key)
                         << " not previously seen before.";
        new_values_.insert(key, pose);
      } else {
        new_values_.update(key, pose);
      }
    } else {
      values_.update(key, pose);
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

  if (key_to == key_from + 1) {
    updatePoseGraphInitialGuess(key_from, key_to, meas);
  }
}

bool DeformationGraph::checkNewBetween(const gtsam::Key& key_from,
                                       const gtsam::Key& key_to) {
  const char& from_prefix = gtsam::Symbol(key_from).chr();
  const char& to_prefix = gtsam::Symbol(key_to).chr();
  const size_t& from_idx = gtsam::Symbol(key_from).index();
  const size_t& to_idx = gtsam::Symbol(key_to).index();

  if (from_idx >= pg_initial_poses_[from_prefix].size()) {
    SPARK_LOG(ERROR)
        << "DeformationGraph: when adding new between from key should already exist.";
    return false;
  }

  if (to_idx > pg_initial_poses_[to_prefix].size()) {
    SPARK_LOG(ERROR) << "DeformationGraph: skipping keys in addNewBetween.";
    return false;
  }

  return true;
}

void DeformationGraph::updatePoseGraphInitialGuess(const gtsam::Key& key_from,
                                                   const gtsam::Key& key_to,
                                                   const gtsam::Pose3& meas) {
  const char& to_prefix = gtsam::Symbol(key_to).chr();
  gtsam::Pose3 initial_estimate;
  gtsam::Pose3 init_pose;
  if (values_.exists(key_from)) {
    initial_estimate = values_.at<gtsam::Pose3>(key_from).compose(meas);
  } else if (new_values_.exists(key_from)) {
    initial_estimate = new_values_.at<gtsam::Pose3>(key_from).compose(meas);
  } else {
    SPARK_LOG(FATAL) << "Missing from key values when adding initial guess.";
  }
  new_values_.insert(key_to, initial_estimate);
  init_pose = pg_initial_poses_[to_prefix].back().compose(meas);
  pg_initial_poses_[to_prefix].push_back(init_pose);
}

void DeformationGraph::addNewBetween(const gtsam::Key& key_from,
                                     const gtsam::Key& key_to,
                                     const gtsam::Pose3& meas,
                                     double variance,
                                     bool temp) {
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);
  if (temp) {
    new_temp_factors_.add(
        gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));
    return;
  }
  new_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  // if it's a loop closure factor
  if (key_to != key_from + 1) {
    SPARK_LOG(INFO) << "DeformationGraph: Added loop closure";
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
                                           const gtsam::Key& key_to) {
  if (!values_.exists(key_from) && !new_values_.exists(key_from) &&
      !new_temp_values_.exists(key_from)) {
    SPARK_LOG(ERROR) << "Key does not exist when adding temporary between factor";
    return false;
  }

  if (!values_.exists(key_to) && !new_values_.exists(key_to) &&
      !new_temp_values_.exists(key_to)) {
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
  char node_prefix = gtsam::Symbol(node_key).chr();
  size_t node_idx = gtsam::Symbol(node_key).index();
  if (!vertex_positions_.count(node_prefix)) {
    vertex_positions_[node_prefix] = std::vector<gtsam::Point3>();
    vertex_stamps_[node_prefix] = std::vector<Timestamp>();
  }
  if (node_idx != vertex_positions_.at(node_prefix).size()) {
    return false;
  }
  vertex_positions_[node_prefix].push_back(node_pose.translation());
  vertex_stamps_[node_prefix].push_back(node_stamp);
  new_values_.insert(node_key, node_pose);
  return true;
}

bool DeformationGraph::checkNewMeshNode(const gtsam::Key& node_key) {
  char node_prefix = gtsam::Symbol(node_key).chr();
  size_t node_idx = gtsam::Symbol(node_key).index();

  if (!vertex_positions_.count(node_prefix)) {
    if (node_idx == 0) {
      return true;
    }
    return false;
  }
  // The check here returns true even for duplicated (added) nodes
  // Only return false if there is a likely message drop
  return node_idx <= vertex_positions_.at(node_prefix).size();
}

bool DeformationGraph::checkNewMeshEdge(const gtsam::Key& from, const gtsam::Key& to) {
  const gtsam::Symbol from_symb(from);
  const gtsam::Symbol to_symb(to);
  if (from_symb.index() >= vertex_positions_.at(from_symb.chr()).size() ||
      to_symb.index() >= vertex_positions_.at(to_symb.chr()).size()) {
    return false;
  }
  if ((!values_.exists(from) && !new_values_.exists(from)) ||
      (!values_.exists(to) && !new_values_.exists(to))) {
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
    const gtsam::Pose3& pose_from = mesh_nodes.at<gtsam::Pose3>(e.first);
    const gtsam::Point3& point_to = mesh_nodes.at<gtsam::Pose3>(e.second).translation();

    addDeformationEdge(e.first, e.second, pose_from, point_to, variance);
  }
}

void DeformationGraph::processNewNode(const gtsam::Key& key,
                                      const gtsam::Pose3& initial_pose,
                                      bool add_prior,
                                      double prior_variance) {
  if (!checkNewNode(key)) {
    SPARK_LOG(FATAL) << "processNewNode failed check.";
  }

  addNewNode(key, initial_pose);

  if (add_prior) {
    addPrior(key, initial_pose, prior_variance);
  }
}

bool DeformationGraph::checkNewNode(const gtsam::Key& key) {
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  if (idx > 0) {
    if (idx != pg_initial_poses_[prefix].size()) {
      SPARK_LOG(ERROR) << "DeformationGraph: Nodes skipped in pose graph nodes";
      return false;
    }
  }
  return true;
}

void DeformationGraph::addNewNode(const gtsam::Key& key,
                                  const gtsam::Pose3& initial_pose) {
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  if (idx == 0) {
    pg_initial_poses_[prefix] = std::vector<gtsam::Pose3>{initial_pose};
  } else {
    pg_initial_poses_[prefix].push_back(initial_pose);
  }

  new_values_.insert(key, initial_pose);
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
  new_temp_values_.insert(key, initial_pose);
}

void DeformationGraph::processNewTempNodesValences(const NodeValenceInfoList& info,
                                                   bool add_prior,
                                                   double edge_variance,
                                                   double prior_variance) {
  for (const auto& factor : info) {
    processNewTempNode(factor.key, factor.pose, add_prior, prior_variance);
    processNodeValence(
        factor.key, factor.valence, factor.valence_prefix, edge_variance, true);
  }
}

void DeformationGraph::removePriorsWithPrefix(const char& prefix) {
  pgo_->removePriorFactorsWithPrefix(prefix);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  recalculate_vertices_ = true;
  return;
}

pcl::PolygonMesh DeformationGraph::deformMesh(const pcl::PolygonMesh& original_mesh,
                                              const std::vector<Timestamp>& stamps,
                                              const std::vector<int>& graph_indices,
                                              const char& prefix,
                                              size_t k,
                                              double tol_t) {
  return deformMesh(original_mesh, stamps, graph_indices, prefix, values_, k, tol_t);
}

pcl::PolygonMesh DeformationGraph::deformMesh(const pcl::PolygonMesh& original_mesh,
                                              const std::vector<Timestamp>& stamps,
                                              const std::vector<int>& graph_indices,
                                              const char& prefix,
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

std::vector<gtsam::Pose3> DeformationGraph::getOptimizedTrajectory(char prefix) const {
  // return the optimized trajectory (pose graph)
  std::vector<gtsam::Pose3> optimized_traj;

  for (size_t i = 0; i < pg_initial_poses_.at(prefix).size(); i++) {
    gtsam::Symbol node(prefix, i);
    if (!values_.exists(node)) {
      break;
    }
    optimized_traj.push_back(values_.at<gtsam::Pose3>(node));
  }
  return optimized_traj;
}

std::vector<gtsam::Pose3> DeformationGraph::getQueuedTrajectory(char prefix) const {
  // return the optimized trajectory (pose graph)
  std::vector<gtsam::Pose3> optimized_traj;

  for (size_t i = 0; i < pg_initial_poses_.at(prefix).size(); i++) {
    gtsam::Symbol node(prefix, i);
    if (values_.exists(node)) {
      optimized_traj.push_back(values_.at<gtsam::Pose3>(node));
    } else if (new_values_.exists(node)) {
      optimized_traj.push_back(new_values_.at<gtsam::Pose3>(node));
    }
  }
  return optimized_traj;
}

void DeformationGraph::getGtsamTempValuesFiltered(gtsam::Values* values,
                                                  const gtsam::Key& min,
                                                  const gtsam::Key& max) const {
  assert(nullptr != values);
  for (const auto& key_value : temp_values_) {
    if (key_value.key >= min && key_value.key < max) {
      values->insert(key_value.key, key_value.value);
    }
  }
}

void DeformationGraph::getGtsamTempFactorsFiltered(gtsam::NonlinearFactorGraph* nfg,
                                                   const gtsam::Key& min,
                                                   const gtsam::Key& max) const {
  assert(nullptr != nfg);
  for (const auto& f : temp_nfg_) {
    bool in_range = false;
    for (const auto& k : f->keys()) {
      if (k > max) {
        in_range = false;
        break;
      } else if (k >= min) {
        in_range = true;
      }
    }
    if (in_range) {
      nfg->add(f);
    }
  }
}

void DeformationGraph::optimize() {
  pgo_->updateTempFactorsValues(new_temp_factors_, new_temp_values_);
  pgo_->forceUpdate(new_factors_, new_values_);
  if (force_recalculate_) {
    recalculate_vertices_ = true;
  }

  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  gnc_weights_ = pgo_->getGncWeights();
  temp_values_ = pgo_->getTempValues();
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_ = gtsam::Values();
  new_temp_factors_ = gtsam::NonlinearFactorGraph();
  new_temp_values_ = gtsam::Values();
}

void DeformationGraph::update() {
  pgo_->updateTempFactorsValues(new_temp_factors_, new_temp_values_);
  pgo_->update(new_factors_, new_values_);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  gnc_weights_ = pgo_->getGncWeights();
  temp_values_ = pgo_->getTempValues();
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  new_factors_ = gtsam::NonlinearFactorGraph();
  new_values_ = gtsam::Values();
}

void DeformationGraph::updateValues(const gtsam::Values& updates) {
  pgo_->updateValues(updates);
  values_ = pgo_->calculateEstimate();
  temp_values_ = pgo_->getTempValues();
}

void DeformationGraph::setParams(const KimeraRPGO::RobustSolverParams& params) {
  pgo_params_ = params;
  pgo_.reset(new KimeraRPGO::RobustSolver(pgo_params_));
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
                                                   int gnc_idx,
                                                   PoseGraphEdge& edge) const {
  // check if prior factor
  const auto factor_ptr = dynamic_cast<const PosePrior*>(factor);
  if (!factor_ptr) {
    return false;
  }

  const gtsam::Symbol key(factor_ptr->key());
  edge.key_from = key.index();
  edge.key_to = key.index();
  edge.robot_from = robot_prefix_to_id.at(key.chr());
  edge.robot_to = robot_prefix_to_id.at(key.chr());
  const auto stamp_ns = maybeGetTimestamp(timestamps, edge.robot_to, edge.key_to);
  if (stamp_ns) {
    edge.stamp_ns = *stamp_ns;
  }

  edge.pose = factor_ptr->prior().matrix();
  if (gnc_idx >= 0 && gnc_weights_.size() > gnc_idx && gnc_weights_(gnc_idx) < 0.5) {
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
    int gnc_idx,
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
    if (gnc_idx >= 0 && gnc_weights_.size() > gnc_idx && gnc_weights_(gnc_idx) < 0.5) {
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
    edge.stamp_ns = vertex_stamps_.at(front.chr()).at(front.index());
  } else if (vertex_prefix_to_id.count(front.chr())) {
    edge.type = EdgeType::MESH_POSE;
    edge.robot_from = vertex_prefix_to_id.at(front.chr());
    edge.robot_to = robot_prefix_to_id.at(back.chr());
    edge.stamp_ns = vertex_stamps_.at(front.chr()).at(front.index());
  } else if (vertex_prefix_to_id.count(back.chr())) {
    edge.type = EdgeType::POSE_MESH;
    edge.robot_from = robot_prefix_to_id.at(front.chr());
    edge.robot_to = vertex_prefix_to_id.at(back.chr());
    edge.stamp_ns = vertex_stamps_.at(back.chr()).at(back.index());
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
      optimized ? values_.at<gtsam::Pose3>(key).matrix()
                : pg_initial_poses_.at(node_symb.chr()).at(node_symb.index()).matrix();
  return true;
}

size_t DeformationGraph::getRemappedId(const std::map<size_t, size_t>& remap,
                                       size_t original) const {
  const auto iter = remap.find(original);
  return iter == remap.end() ? original : iter->second;
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
  node.stamp_ns = vertex_stamps_.at(node_symb.chr()).at(node_symb.index());
  node.pose =
      optimized
          ? values_.at<gtsam::Pose3>(key).matrix()
          : gtsam::Pose3(gtsam::Rot3(),
                         vertex_positions_.at(node_symb.chr()).at(node_symb.index()))
                .matrix();
  return true;
}

PoseGraph::Ptr DeformationGraph::getPoseGraph(const RobotTimestampMap& timestamps,
                                              bool include_deformation_edges,
                                              bool include_between_edges,
                                              bool optimized) const {
  auto graph = std::make_shared<PoseGraph>();

  // first store the factors as edges
  for (size_t i = 0; i < nfg_.size(); i++) {
    PoseGraphEdge pg_edge;
    if (include_between_edges &&
        tryConvertFactorToBetweenEdge(nfg_[i].get(), timestamps, i, pg_edge)) {
      graph->edges.push_back(pg_edge);
    }

    if (include_between_edges &&
        tryConvertFactorToPriorEdge(nfg_[i].get(), timestamps, i, pg_edge)) {
      graph->edges.push_back(pg_edge);
    }

    if (include_deformation_edges &&
        tryConvertFactorToDeformationEdge(nfg_[i].get(), pg_edge)) {
      graph->edges.push_back(pg_edge);
    }
  }

  // Then store the values as nodes
  for (const auto& key : values_.keys()) {
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

}  // namespace kimera_pgmo
