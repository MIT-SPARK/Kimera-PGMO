/**
 * @file   deformation_graph.cpp
 * @brief  DeformationGraph class
 * @author Yun Chang
 */
#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include "kimera_pgmo/deformation_graph.h"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <kimera_rpgo/pose_graph_4dof.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/common/io.h>

#include <cmath>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/pcl_mesh_traits.h"
#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

using MeasurementVector = std::vector<std::pair<gtsam::Key, gtsam::Pose3>>;
using RobotTimestampMap = std::map<size_t, std::vector<Timestamp>>;
using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;
using pose_graph_tools::PoseGraphNode;
using PoseBetween = gtsam::BetweenFactor<gtsam::Pose3>;
using PosePrior = gtsam::PriorFactor<gtsam::Pose3>;
using EdgeType = pose_graph_tools::PoseGraphEdge::Type;

DeformationGraph::DeformationGraph(bool add_init_vertex_prior, PoseMode mode)
    : add_init_vertex_prior_(add_init_vertex_prior),
      verbose_(true),
      values_(new gtsam::Values),
      temp_values_(new gtsam::Values),
      pose_mode_(mode),
      recalculate_vertices_(false) {}

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
    if (checkNewNode(node_key)) {
      addNewNode(node_key, node_pose);
    }

    setPoseTimestamp(node_key, node.stamp_ns);
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
                                          const std::vector<uint64_t>& valences,
                                          const char& valence_prefix,
                                          double variance,
                                          bool temp) {
  // Add the consistency factors
  for (const auto& v : valences) {
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
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  const char& valence_prefix = gtsam::Symbol(vertex).chr();
  const size_t& valence_idx = gtsam::Symbol(vertex).index();
  if (!values_->exists(vertex) && !temp_values_->exists(vertex)) {
    return false;
  }
  bool non_temp_node =
      pg_initial_poses_.count(prefix) && pg_initial_poses_.at(prefix).size() > idx;
  node_pose = non_temp_node ? pg_initial_poses_.at(prefix).at(idx)
                            : temp_pg_initial_poses_.at(key);
  vertex_pos = vertex_positions_.at(valence_prefix).at(valence_idx);
  return true;
}

void DeformationGraph::processBetweenAsMeshConnections(
    const gtsam::Pose3& w_T_source,
    const std::vector<uint64_t>& source,
    const gtsam::Pose3& w_T_dest,
    const std::vector<uint64_t>& dest,
    const gtsam::Pose3& source_T_dest,
    const char& source_prefix,
    const char& dest_prefix,
    double variance,
    bool temp,
    bool known_inliers) {
  for (const auto& s : source) {
    const gtsam::Symbol vertex_s(source_prefix, s);
    auto w_T_s = gtsam::Pose3(gtsam::Rot3(), vertex_positions_[source_prefix].at(s));
    auto s_T_source = w_T_s.between(w_T_source);
    for (const auto& d : dest) {
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
      auto w_T_d = gtsam::Pose3(gtsam::Rot3(), vertex_positions_[dest_prefix].at(d));
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
  if (!values_->exists(to_key) && !temp_values_->exists(to_key)) {
    if (temp) {
      insertValue(to_key, gtsam::Pose3(gtsam::Rot3(), to_point), true);
    } else {
      insertValue(to_key, gtsam::Pose3(gtsam::Rot3(), to_point));
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
  addDeformationEdge(
      from_key, to_key, from_pose.transformTo(to_point), variance, temp, known_inlier);
}

void DeformationGraph::addDeformationEdge(const gtsam::Key& from_key,
                                          const gtsam::Key& to_key,
                                          const gtsam::Point3& measurement,
                                          double variance,
                                          bool temp,
                                          bool known_inlier) {
  const auto noise = gtsam::noiseModel::Isotropic::Variance(3, variance);
  appendFactor(
      makeDeformationFactor(from_key, to_key, measurement, noise), temp, known_inlier);
  adjacency_map_[from_key].insert(to_key);
}

void DeformationGraph::addPrior(const gtsam::Key& key,
                                const gtsam::Pose3& pose,
                                double variance,
                                bool temp,
                                bool known_inlier) {
  appendFactor(makePrior(key, pose, variance), temp, known_inlier);
}

void DeformationGraph::processNodeMeasurements(const MeasurementVector& measurements,
                                               double variance) {
  gtsam::Values updates;
  for (const auto& [key, pose] : measurements) {
    if (!values_->exists(key)) {
      insertValue(key, pose);
    } else if (updates.exists(key)) {
      updates.update(key, pose);
    } else {
      updates.insert(key, pose);
    }

    addPrior(key, pose, variance);
  }

  updateValues(updates);
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
  const char& from_prefix = gtsam::Symbol(key_from).chr();
  const char& to_prefix = gtsam::Symbol(key_to).chr();
  const size_t& from_idx = gtsam::Symbol(key_from).index();
  const size_t& to_idx = gtsam::Symbol(key_to).index();

  // Check if prefixes exist in pg_initial_poses_
  if (!pg_initial_poses_.count(from_prefix)) {
    return false;
  }

  if (!pg_initial_poses_.count(to_prefix)) {
    return false;
  }

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
  const char& to_prefix = gtsam::Symbol(key_to).chr();
  gtsam::Pose3 initial_estimate, init_pose;
  if (values_->exists(key_from)) {
    initial_estimate = values_->at<gtsam::Pose3>(key_from).compose(meas);
  } else {
    SPARK_LOG(FATAL) << "Missing from key values when adding initial guess.";
  }

  init_pose = originalPose(key_from).compose(meas);
  insertValue(key_to, init_pose);
  gtsam::Values updates;
  updates.insert(key_to, initial_estimate);
  updateValues(updates);
  pg_initial_poses_[to_prefix].push_back(init_pose);
}

void DeformationGraph::addNewBetween(const gtsam::Key& key_from,
                                     const gtsam::Key& key_to,
                                     const gtsam::Pose3& meas,
                                     double variance,
                                     bool temp,
                                     bool known_inlier) {
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-2 * variance);
  variances.tail<3>().setConstant(variance);
  const auto noise = gtsam::noiseModel::Diagonal::Variances(variances);
  if (isPose4(key_from) != isPose4(key_to)) {
    throw std::invalid_argument("Between factors require matching pose types");
  }

  if (isPose4(key_from)) {
    const auto& source = originalPose(key_from);
    appendFactor(boost::make_shared<kimera_rpgo::Pose4BetweenFactor>(
                     key_from,
                     key_to,
                     kimera_rpgo::projectBetween(source, meas),
                     kimera_rpgo::projectBetweenNoise(source, meas, noise)),
                 temp,
                 known_inlier);
  } else {
    appendFactor(boost::make_shared<PoseBetween>(key_from, key_to, meas, noise),
                 temp,
                 known_inlier);
  }

  if (!temp && key_to != key_from + 1) {
    ++num_loopclosures_;
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
  if (!values_->exists(key_from) && !temp_values_->exists(key_from)) {
    SPARK_LOG(ERROR) << "Key does not exist when adding temporary between factor";
    return false;
  }

  if (!values_->exists(key_to) && !temp_values_->exists(key_to)) {
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

  mesh_keys_.insert(node_key);
  vertex_positions_[node_prefix].push_back(node_pose.translation());
  vertex_stamps_[node_prefix].push_back(node_stamp);
  insertValue(node_key, node_pose);
  // Add a prior to the first mesh-only control.
  if (add_init_vertex_prior_ && values_->size() == 1) {
    addPrior(node_key, node_pose, 1e-3, false, true);
  }

  return true;
}

bool DeformationGraph::checkNewMeshNode(const gtsam::Key& node_key) const {
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

bool DeformationGraph::checkNewMeshEdge(const gtsam::Key& from,
                                        const gtsam::Key& to) const {
  const gtsam::Symbol from_symb(from);
  const gtsam::Symbol to_symb(to);
  if (from_symb.index() >= vertex_positions_.at(from_symb.chr()).size() ||
      to_symb.index() >= vertex_positions_.at(to_symb.chr()).size()) {
    return false;
  }
  if (!values_->exists(from) || !values_->exists(to)) {
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
  const char prefix = gtsam::Symbol(key).chr();
  const size_t idx = gtsam::Symbol(key).index();

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
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  if (idx == 0) {
    pg_initial_poses_[prefix] = std::vector<gtsam::Pose3>{initial_pose};
  } else {
    pg_initial_poses_[prefix].push_back(initial_pose);
  }

  insertValue(key, initial_pose);
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
  insertValue(key, initial_pose, true);
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
    for (const auto v : factor.valence) {
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

void DeformationGraph::removePriorsWithPrefix(const char& prefix) {
  const auto keep = [&](const gtsam::NonlinearFactor& factor) {
    const bool prior =
        dynamic_cast<const PosePrior*>(&factor) ||
        dynamic_cast<const gtsam::PriorFactor<gtsam::Pose4DoF>*>(&factor);
    return !prior || gtsam::Symbol(factor.front()).chr() != prefix;
  };
  permanent_.filter(keep);
  temporary_.filter(keep);
  rebuildBookkeeping();
}

pcl::PolygonMesh DeformationGraph::deformMesh(const pcl::PolygonMesh& original_mesh,
                                              const std::vector<Timestamp>& stamps,
                                              const std::vector<int>& graph_indices,
                                              const char& prefix,
                                              size_t k,
                                              double tol_t) {
  return deformMesh(original_mesh, stamps, graph_indices, prefix, *values_, k, tol_t);
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

std::vector<gtsam::Pose3> DeformationGraph::getTrajectory(char prefix) const {
  // return the optimized trajectory (pose graph)
  std::vector<gtsam::Pose3> traj;

  for (size_t i = 0; i < pg_initial_poses_.at(prefix).size(); i++) {
    gtsam::Symbol node(prefix, i);
    if (!values_->exists(node)) {
      break;
    }
    traj.push_back(values_->at<gtsam::Pose3>(node));
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
  const auto prior3 = dynamic_cast<const PosePrior*>(factor);
  const auto prior4 = dynamic_cast<const gtsam::PriorFactor<gtsam::Pose4DoF>*>(factor);
  if (!prior3 && !prior4) {
    return false;
  }

  const auto factor_ptr = dynamic_cast<const gtsam::NoiseModelFactor*>(factor);
  const gtsam::Symbol key(factor->front());
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

  edge.pose = prior3 ? prior3->prior().matrix() : prior4->prior().pose().matrix();
  if (permanent_.inlier_weights.empty()) {
    // If no inlier weights assume no outlier rejection
    edge.type = EdgeType::PRIOR;
  } else {
    if (factor_idx >= 0 && permanent_.inlier_weights.size() > factor_idx &&
        permanent_.inlier_weights.at(factor_idx) < 0.5) {
      edge.type = EdgeType::REJECTED_PRIOR;
    } else {
      edge.type = EdgeType::PRIOR;
    }
  }

  const auto noise_model =
      dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor_ptr->noiseModel().get());
  if (noise_model) {
    if (prior3) {
      edge.covariance = noise_model->covariance();
    } else {
      const std::function<gtsam::Pose3(const gtsam::Pose4DoF&)> lift =
          [](const auto& pose) { return pose.pose(); };
      const auto J = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Pose4DoF>(
          lift, prior4->prior());
      edge.covariance = J * noise_model->covariance() * J.transpose();
    }
  }

  return true;
}

bool DeformationGraph::tryConvertFactorToBetweenEdge(
    gtsam::NonlinearFactor* factor,
    const RobotTimestampMap& timestamps,
    int factor_idx,
    PoseGraphEdge& edge) const {
  // check if between factor
  const auto between3 = dynamic_cast<const PoseBetween*>(factor);
  const auto between4 =
      dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose4DoF>*>(factor);
  if (!between3 && !between4) {
    return false;
  }

  const auto factor_ptr = dynamic_cast<const gtsam::NoiseModelFactor*>(factor);
  const gtsam::Symbol front(factor->front());
  const gtsam::Symbol back(factor->back());
  if (!robot_prefix_to_id.count(front.chr()) || !robot_prefix_to_id.count(back.chr())) {
    return false;
  }

  const std::function<gtsam::Pose3(const gtsam::Pose4DoF&)> lift =
      [&](const auto& relative) {
        const auto& source = originalPose(front);
        const auto source_yaw = source.rotation().yaw();
        const auto target_tilt = originalPose(back).rotation().ypr();
        const gtsam::Pose3 target(
            gtsam::Rot3::Ypr(
                source_yaw + relative.yaw(), target_tilt(1), target_tilt(2)),
            source.translation() +
                gtsam::Rot3::Yaw(source_yaw).rotate(relative.translation()));
        return source.between(target);
      };
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
    if (permanent_.inlier_weights.empty()) {
      // If no inlier weights assume no outlier rejection
      edge.type = EdgeType::LOOPCLOSE;
    } else {
      if (factor_idx >= 0 && permanent_.inlier_weights.size() > factor_idx &&
          permanent_.inlier_weights.at(factor_idx) < 0.5) {
        edge.type = EdgeType::REJECTED_LOOPCLOSE;
      } else {
        edge.type = EdgeType::LOOPCLOSE;
      }
    }
  }

  edge.pose =
      between3 ? between3->measured().matrix() : lift(between4->measured()).matrix();

  const auto noise_model =
      dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor_ptr->noiseModel().get());
  if (noise_model) {
    if (between3) {
      edge.covariance = noise_model->covariance();
    } else {
      const auto J = gtsam::numericalDerivative11<gtsam::Pose3, gtsam::Pose4DoF>(
          lift, between4->measured());
      edge.covariance = J * noise_model->covariance() * J.transpose();
    }
  }

  return true;
}

bool DeformationGraph::tryConvertFactorToDeformationEdge(gtsam::NonlinearFactor* factor,
                                                         PoseGraphEdge& edge) const {
  const auto f33 = dynamic_cast<const DeformationEdgeFactor*>(factor);
  const auto f34 = dynamic_cast<const DeformationEdgeFactorToPose4DoF*>(factor);
  const auto f43 = dynamic_cast<const DeformationEdgeFactorFromPose4DoF*>(factor);
  const auto f44 = dynamic_cast<const DeformationEdgeFactor4DoF*>(factor);
  if (!f33 && !f34 && !f43 && !f44) {
    return false;
  }

  const auto measurement = f33   ? f33->measurement()
                           : f34 ? f34->measurement()
                           : f43 ? f43->measurement()
                                 : f44->measurement();
  const auto factor_ptr = dynamic_cast<const gtsam::NoiseModelFactor*>(factor);
  const gtsam::Symbol front(factor->front());
  const gtsam::Symbol back(factor->back());
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

  edge.pose = gtsam::Pose3(gtsam::Rot3(), measurement).matrix();

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
      optimized ? values_->at<gtsam::Pose3>(key).matrix()
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
  node.stamp_ns = vertex_stamps_.at(node_symb.chr()).at(node_symb.index());
  node.pose =
      optimized
          ? values_->at<gtsam::Pose3>(key).matrix()
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
  size_t index = 0;
  for (const auto& factor : permanent_.factors) {
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
  for (const auto& key : values_->keys()) {
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
  updateState(updates, false);
}

void DeformationGraph::updateTempValues(const gtsam::Values& updates) {
  updateState(updates, true);
}

void DeformationGraph::updateInlierWeights(const std::vector<double>& weights) {
  if (!weights.empty() && weights.size() != permanent_.factors.size()) {
    throw std::invalid_argument("Incorrect permanent weight count");
  }

  permanent_.inlier_weights = weights;
}

void DeformationGraph::updateTempInlierWeights(const std::vector<double>& weights) {
  if (!weights.empty() && weights.size() != temporary_.factors.size()) {
    throw std::invalid_argument("Incorrect temporary weight count");
  }

  temporary_.inlier_weights = weights;
}

}  // namespace kimera_pgmo
