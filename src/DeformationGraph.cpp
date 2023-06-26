/**
 * @file   DeformationGraph.cpp
 * @brief  DeformationGraph class
 * @author Yun Chang
 */
#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <algorithm>
#include <cmath>
#include <numeric>

#include "kimera_pgmo/DeformationGraph.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using pcl::PolygonMesh;

namespace kimera_pgmo {

DeformationGraph::DeformationGraph()
    : verbose_(true),
      pgo_(nullptr),
      recalculate_vertices_(false),
      force_recalculate_(true) {}
DeformationGraph::~DeformationGraph() {}

bool DeformationGraph::initialize(const KimeraRPGO::RobustSolverParams& params) {
  // Initialize RPGO
  pgo_params_ = params;
  pgo_ =
      std::unique_ptr<KimeraRPGO::RobustSolver>(new KimeraRPGO::RobustSolver(params));
  return true;
}

void DeformationGraph::addNodeValence(const gtsam::Key& key,
                                      const Vertices& valences,
                                      const char& valence_prefix,
                                      double variance) {
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  // Add the consistency factors
  for (Vertex v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);
    if (!values_.exists(vertex) && !new_values_.exists(vertex)) {
      continue;
    }
    const gtsam::Pose3& node_pose = pg_initial_poses_[prefix].at(idx);
    const gtsam::Pose3 vertex_pose(gtsam::Rot3(),
                                   vertex_positions_[valence_prefix].at(v));

    // Define noise. Hardcoded for now
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(3, variance);
    // Create deformation edge factor
    const DeformationEdgeFactor new_edge_1(
        key, vertex, node_pose, vertex_pose.translation(), noise);
    const DeformationEdgeFactor new_edge_2(
        vertex, key, vertex_pose, node_pose.translation(), noise);
    consistency_factors_.add(new_edge_1);
    consistency_factors_.add(new_edge_2);
    new_factors_.add(new_edge_1);
    new_factors_.add(new_edge_2);
  }
}

void DeformationGraph::addTempNodeValence(const gtsam::Key& key,
                                          const Vertices& valences,
                                          const char& valence_prefix,
                                          double variance) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  // Add the consistency factors
  for (Vertex v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);
    if (!values_.exists(vertex) and !new_values_.exists(vertex)) continue;

    const gtsam::Pose3& node_pose = temp_pg_initial_poses_.at(key);
    const gtsam::Pose3 vertex_pose(gtsam::Rot3(),
                                   vertex_positions_[valence_prefix].at(v));

    // Define noise. Hardcoded for now
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(3, variance);
    // Create deformation edge factor
    const DeformationEdgeFactor new_edge_1(
        key, vertex, node_pose, vertex_pose.translation(), noise);
    const DeformationEdgeFactor new_edge_2(
        vertex, key, vertex_pose, node_pose.translation(), noise);
    new_factors.add(new_edge_1);
    new_factors.add(new_edge_2);
  }
  pgo_->updateTempFactorsValues(new_factors, new_values);
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  temp_values_ = pgo_->getTempValues();
}

void DeformationGraph::addMeasurement(const Vertex& v,
                                      const geometry_msgs::Pose& pose,
                                      const char& prefix,
                                      double variance) {
  // noise for measurement
  // TODO(yun) take in t_variance and r_variance? Though adds more parameters...
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);

  gtsam::Symbol v_symb(prefix, v);
  gtsam::Pose3 meas = RosToGtsam(pose);
  gtsam::PriorFactor<gtsam::Pose3> absolute_meas(v_symb, meas, noise);
  new_factors_.add(absolute_meas);
}

void DeformationGraph::addNodeMeasurements(
    const std::vector<std::pair<gtsam::Key, gtsam::Pose3>>& measurements,
    double variance) {
  for (auto keyed_pose : measurements) {
    if (!values_.exists(keyed_pose.first)) {
      ROS_ERROR_STREAM("DeformationGraph: adding node measurement to a node "
                       << gtsam::DefaultKeyFormatter(keyed_pose.first)
                       << " not previously seen before.");
      if (!new_values_.exists(keyed_pose.first)) {
        new_values_.insert(keyed_pose.first, keyed_pose.second);
      } else {
        new_values_.update(keyed_pose.first, keyed_pose.second);
      }
    } else {
      values_.update(keyed_pose.first, keyed_pose.second);
    }
    gtsam::Vector6 variances;
    variances.head<3>().setConstant(1e-02 * variance);
    variances.tail<3>().setConstant(variance);
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Diagonal::Variances(variances);
    gtsam::PriorFactor<gtsam::Pose3> measurement(
        keyed_pose.first, keyed_pose.second, noise);
    new_factors_.add(measurement);
  }
}

void DeformationGraph::addNewBetween(const gtsam::Key& key_from,
                                     const gtsam::Key& key_to,
                                     const gtsam::Pose3& meas,
                                     const gtsam::Pose3& initial_pose,
                                     double variance) {
  const char& from_prefix = gtsam::Symbol(key_from).chr();
  const char& to_prefix = gtsam::Symbol(key_to).chr();
  const size_t& from_idx = gtsam::Symbol(key_from).index();
  const size_t& to_idx = gtsam::Symbol(key_to).index();

  if (from_idx >= pg_initial_poses_[from_prefix].size()) {
    ROS_ERROR(
        "DeformationGraph: when adding new between from key should already "
        "exist.");
    return;
  }

  if (to_idx > pg_initial_poses_[to_prefix].size()) {
    ROS_ERROR("DeformationGraph: skipping keys in addNewBetween.");
    return;
  } else if (to_idx == pg_initial_poses_[to_prefix].size()) {
    // new node
    // For now push empty valence, valences will be populated when updated
    Vertices valences;
    pg_initial_poses_[to_prefix].push_back(initial_pose);

    gtsam::Pose3 initial_estimate = initial_pose;
    if (to_idx > 0) {
      if (values_.exists(key_from)) {
        initial_estimate = values_.at<gtsam::Pose3>(key_from).compose(meas);
      } else if (new_values_.exists(key_from)) {
        initial_estimate = new_values_.at<gtsam::Pose3>(key_from).compose(meas);
      }
    }
    new_values_.insert(key_to, initial_estimate);
  }

  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);
  new_factors_.add(gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  // if it's a loop closure factor
  if (key_to != key_from + 1) {
    ROS_INFO("DeformationGraph: Added loop closure. ");
    recalculate_vertices_ = true;
  }
  return;
}

void DeformationGraph::addNewTempBetween(const gtsam::Key& key_from,
                                         const gtsam::Key& key_to,
                                         const gtsam::Pose3& meas,
                                         const gtsam::Pose3& initial_pose,
                                         double variance) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
  if (!values_.exists(key_from) && !new_values_.exists(key_from) &&
      !temp_values_.exists(key_from)) {
    ROS_ERROR("Key does not exist when adding temporary between factor. ");
    return;
  }

  if (!values_.exists(key_to) && !new_values_.exists(key_to) &&
      !temp_values_.exists(key_to)) {
    ROS_ERROR("Key does not exist when adding temporary between factor. ");
    return;
  }

  // Note that unlike the typical addNewBetween, this one only adds the
  // temporary between factors without any values
  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);
  new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  pgo_->updateTempFactorsValues(new_factors, new_values);
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  temp_values_ = pgo_->getTempValues();
  return;
}

// TODO(yun) add unittest
void DeformationGraph::addNewTempEdges(const pose_graph_tools::PoseGraph& edges,
                                       double variance,
                                       bool rotations_known) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  gtsam::Vector6 variances;
  if (rotations_known) {
    variances.head<3>().setConstant(1e-02 * variance);
  } else {
    variances.head<3>().setConstant(1e+08);
  }
  variances.tail<3>().setConstant(variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);

  for (const auto& e : edges.edges) {
    if (!values_.exists(e.key_from) && ~new_values_.exists(e.key_from) &&
        !temp_values_.exists(e.key_from)) {
      ROS_ERROR("Key does not exist when adding temporary between factor. ");
      continue;
    }

    if (!values_.exists(e.key_to) && ~new_values_.exists(e.key_to) &&
        !temp_values_.exists(e.key_to)) {
      ROS_ERROR("Key does not exist when adding temporary between factor. ");
      continue;
    }

    new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
        e.key_from, e.key_to, RosToGtsam(e.pose), noise));
  }

  // Note that unlike the typical addNewBetween, this one only adds the
  // temporary between factors without any values
  pgo_->updateTempFactorsValues(new_factors, new_values);
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  temp_values_ = pgo_->getTempValues();
  return;
}

void DeformationGraph::addNewMeshEdgesAndNodes(
    const std::vector<std::pair<gtsam::Key, gtsam::Key>>& mesh_edges,
    const gtsam::Values& mesh_nodes,
    const std::unordered_map<gtsam::Key, ros::Time>& node_stamps,
    std::vector<size_t>* added_indices,
    std::vector<ros::Time>* added_index_stamps,
    double variance) {
  assert(stamps.size() == mesh_nodes.size());
  // New mesh edge factors
  gtsam::NonlinearFactorGraph new_mesh_factors;
  // New mesh node factors
  gtsam::Values new_mesh_nodes;
  // Iterate and add the new mesh nodes not yet in graph
  // Note that the keys are in increasing order by construction from gtsam
  for (auto k : mesh_nodes.keys()) {
    char node_prefix = gtsam::Symbol(k).chr();
    size_t node_idx = gtsam::Symbol(k).index();
    const gtsam::Pose3& node_pose = mesh_nodes.at<gtsam::Pose3>(k);
    try {
      if (node_idx > vertex_positions_.at(node_prefix).size()) {
        ROS_ERROR_STREAM(
            "Adding new mesh edges and nodes: node index does not match index "
            "in vertex position vector. Likely to have dropped packets from "
            "frontend. "
            << node_idx << " vs. " << vertex_positions_.at(node_prefix).size());
        while (vertex_positions_.at(node_prefix).size() < node_idx) {
          // Place at inifinity to ignore
          vertex_positions_[node_prefix].push_back(gtsam::Point3(0, 0, 0));
          vertex_stamps_[node_prefix].push_back(node_stamps.at(k));
        }
      }
      if (node_idx == vertex_positions_.at(node_prefix).size()) {
        // Only add nodes that has not previously been added
        vertex_positions_[node_prefix].push_back(node_pose.translation());
        vertex_stamps_[node_prefix].push_back(node_stamps.at(k));
        new_mesh_nodes.insert(k, node_pose);
        added_indices->push_back(node_idx);
        added_index_stamps->push_back(node_stamps.at(k));
      }
    } catch (const std::out_of_range& e) {
      if (verbose_) {
        ROS_INFO_STREAM("New prefix "
                        << node_prefix
                        << " detected when adding new mesh edges and nodes. ");
      }
      vertex_positions_[node_prefix] = std::vector<gtsam::Point3>{};
      vertex_stamps_[node_prefix] = std::vector<ros::Time>{};
      vertex_positions_[node_prefix].push_back(node_pose.translation());
      vertex_stamps_[node_prefix].push_back(node_stamps.at(k));
      new_mesh_nodes.insert(k, node_pose);
      added_indices->push_back(node_idx);
      added_index_stamps->push_back(node_stamps.at(k));
    }
  }

  // Define noise. Hardcoded for now
  static const gtsam::SharedNoiseModel& edge_noise =
      gtsam::noiseModel::Isotropic::Variance(3, variance);
  // Iterate and add the new edges
  for (auto e : mesh_edges) {
    const gtsam::Symbol& from = gtsam::Symbol(e.first);
    const gtsam::Symbol& to = gtsam::Symbol(e.second);
    if (from.index() >= vertex_positions_.at(from.chr()).size() ||
        to.index() >= vertex_positions_.at(to.chr()).size())
      continue;
    if ((!values_.exists(from) && !new_values_.exists(from) &&
         !new_mesh_nodes.exists(from)) ||
        (!values_.exists(to) && !new_values_.exists(to) && !new_mesh_nodes.exists(to)))
      continue;
    const gtsam::Pose3& pose_from = mesh_nodes.at<gtsam::Pose3>(from);
    const gtsam::Pose3& pose_to = mesh_nodes.at<gtsam::Pose3>(to);
    // Create new edge as deformation edge factor
    const DeformationEdgeFactor new_edge(
        from, to, pose_from, pose_to.translation(), edge_noise);
    new_mesh_factors.add(new_edge);
    consistency_factors_.add(new_edge);
  }

  new_factors_.add(new_mesh_factors);
  new_values_.insert(new_mesh_nodes);
}

void DeformationGraph::addNewNode(const gtsam::Key& key,
                                  const gtsam::Pose3& initial_pose,
                                  bool add_prior,
                                  double prior_variance) {
  Vertices valences;
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  if (idx == 0) {
    pg_initial_poses_[prefix] = std::vector<gtsam::Pose3>{initial_pose};
  } else {
    if (idx != pg_initial_poses_[prefix].size()) {
      ROS_ERROR("DeformationGraph: Nodes skipped in pose graph nodes. ");
    }
    pg_initial_poses_[prefix].push_back(initial_pose);
  }

  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * prior_variance);
  variances.tail<3>().setConstant(prior_variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);
  new_values_.insert(key, initial_pose);
  if (add_prior) {
    new_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(key, initial_pose, noise));
  }
  return;
}

void DeformationGraph::addNewTempNode(const gtsam::Key& key,
                                      const gtsam::Pose3& initial_pose,
                                      bool add_prior,
                                      double prior_variance) {
  // new temp node
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  temp_pg_initial_poses_[key] = initial_pose;

  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * prior_variance);
  variances.tail<3>().setConstant(prior_variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);
  new_values.insert(key, initial_pose);
  if (add_prior) {
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, initial_pose, noise));
  }

  pgo_->updateTempFactorsValues(new_factors, new_values);
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  temp_values_ = pgo_->getTempValues();
  return;
}

// TODO(yun) add unittests
void DeformationGraph::addNewTempNodesValences(
    const std::vector<gtsam::Key>& keys,
    const std::vector<gtsam::Pose3>& initial_poses,
    const std::vector<Vertices>& valences,
    const char& valence_prefix,
    bool add_prior,
    double edge_variance,
    const std::vector<gtsam::Pose3>& initial_guesses,
    double prior_variance) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  assert(keys.size() == initial_poses.size());
  assert(keys.size() == valences.size());

  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * prior_variance);
  variances.tail<3>().setConstant(prior_variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);

  for (size_t i = 0; i < keys.size(); i++) {
    temp_pg_initial_poses_[keys[i]] = initial_poses[i];
    if (initial_guesses.size() > i) {
      new_values.insert(keys[i], initial_guesses[i]);
    } else {
      new_values.insert(keys[i], initial_poses[i]);
    }
    if (add_prior) {
      new_factors.add(
          gtsam::PriorFactor<gtsam::Pose3>(keys[i], initial_poses[i], noise));
    }

    for (Vertex v : valences[i]) {
      const gtsam::Symbol vertex(valence_prefix, v);
      if (!values_.exists(vertex) && !new_values_.exists(vertex)) continue;

      const gtsam::Pose3& node_pose = initial_poses[i];
      const gtsam::Pose3 vertex_pose(gtsam::Rot3(),
                                     vertex_positions_[valence_prefix].at(v));

      // Define noise. Hardcoded for now
      static const gtsam::SharedNoiseModel& noise =
          gtsam::noiseModel::Isotropic::Variance(3, edge_variance);
      // Create deformation edge factor
      const DeformationEdgeFactor new_edge_1(
          keys[i], vertex, node_pose, vertex_pose.translation(), noise);
      const DeformationEdgeFactor new_edge_2(
          vertex, keys[i], vertex_pose, node_pose.translation(), noise);
      new_factors.add(new_edge_1);
      new_factors.add(new_edge_2);
    }
  }

  pgo_->updateTempFactorsValues(new_factors, new_values);
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  temp_values_ = pgo_->getTempValues();
  return;
}

void DeformationGraph::removePriorsWithPrefix(const char& prefix) {
  pgo_->removePriorFactorsWithPrefix(prefix);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  recalculate_vertices_ = true;
  return;
}

pcl::PolygonMesh DeformationGraph::deformMesh(const pcl::PolygonMesh& original_mesh,
                                              const std::vector<ros::Time>& stamps,
                                              const std::vector<int>& graph_indices,
                                              const char& prefix,
                                              size_t k,
                                              double tol_t) {
  return deformMesh(original_mesh, stamps, graph_indices, prefix, values_, k, tol_t);
}

pcl::PolygonMesh DeformationGraph::deformMesh(const pcl::PolygonMesh& original_mesh,
                                              const std::vector<ros::Time>& stamps,
                                              const std::vector<int>& graph_indices,
                                              const char& prefix,
                                              const gtsam::Values& optimized_values,
                                              size_t k,
                                              double tol_t) {
  // extract original vertices
  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  pcl::fromPCLPointCloud2(original_mesh.cloud, new_vertices);

  // note that there's no aliasing w.r.t. the new / old vertices so this is relatively safe
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

size_t DeformationGraph::findStartIndex(char prefix,
                                        int start_index_hint,
                                        const std::vector<ros::Time>& stamps,
                                        double tol_t) const {
  const bool have_prefix_vertices =
      last_calculated_vertices_.find(prefix) != last_calculated_vertices_.end();
  if (!have_prefix_vertices) {
    return 0;
  }

  if (recalculate_vertices_) {
    ROS_INFO("DeformationGraph: Re-calculating all mesh vertices in deformMesh. ");
    return 0;
  }

  if (start_index_hint >= 0) {
    return start_index_hint;
  }

  double min_time = std::max(0.0, vertex_stamps_.at(prefix).back().toSec() - tol_t);
  const ros::Time min_stamp = ros::Time(min_time);

  auto bound = std::upper_bound(stamps.begin(), stamps.end(), min_stamp);
  return std::min(static_cast<size_t>(bound - stamps.begin()),
                  last_calculated_vertices_.at(prefix).size());
}

void DeformationGraph::predeformPoints(
    pcl::PointCloud<pcl::PointXYZRGBA>& new_vertices,
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const gtsam::Values& optimized_values,
    const std::vector<int>& graph_indices,
    std::vector<size_t>& indices_to_deform,
    char prefix,
    size_t start_idx) {
  for (size_t i = start_idx; i < vertices.size(); i++) {
    const int index = graph_indices.at(i);
    if (index < 0 || !optimized_values.exists(gtsam::Symbol(prefix, index))) {
      // Have to check here because sometimes interpolation happen before mesh
      // graph received
      // TODO(yun) double check this
      indices_to_deform.push_back(i);
      continue;
    }

    const pcl::PointXYZRGBA& p_old = vertices[i];
    gtsam::Point3 vi(p_old.x, p_old.y, p_old.z);

    gtsam::Pose3 node_transform =
        optimized_values.at<gtsam::Pose3>(gtsam::Symbol(prefix, index));
    gtsam::Point3 gindex = vertex_positions_[prefix].at(index);
    gtsam::Point3 deformed_point =
        node_transform.rotation().rotate(vi - gindex) + node_transform.translation();

    auto& p = new_vertices[i];
    p.x = deformed_point.x();
    p.y = deformed_point.y();
    p.z = deformed_point.z();
  }
}

void DeformationGraph::deformPoints(
    pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const pcl::PointCloud<pcl::PointXYZRGBA>& old_vertices,
    const std::vector<ros::Time>& stamps,
    char prefix,
    const gtsam::Values& optimized_values,
    size_t k,
    double tol_t,
    const std::vector<int>* graph_indices,
    int start_index_hint,
    std::vector<std::set<size_t>>* vertex_graph_map) {
  // Cannot deform if no nodes in the deformation graph
  if (vertex_positions_.find(prefix) == vertex_positions_.end()) {
    ROS_DEBUG(
        "Deformation graph has no vertices associated with mesh prefix. No "
        "deformation. ");
    return;
  }

  const size_t start_idx = findStartIndex(prefix, start_index_hint, stamps, tol_t);

  std::vector<size_t> indices_to_deform;
  if (start_idx != 0) {
    const auto& last_vertices = last_calculated_vertices_.at(prefix);
    for (size_t i = 0; i < start_idx; i++) {
      vertices.points[i] = last_vertices.points[i];
    }

    if (graph_indices) {
      predeformPoints(vertices,
                      old_vertices,
                      optimized_values,
                      *graph_indices,
                      indices_to_deform,
                      prefix,
                      start_idx);
    } else {
      indices_to_deform.resize(vertices.size() - start_idx);
      std::iota(indices_to_deform.begin(), indices_to_deform.end(), start_idx);
    }
  }

  const std::vector<size_t>* indices_ptr =
      start_idx == 0 ? nullptr : &indices_to_deform;
  std::vector<std::set<size_t>> vertex_graph_map_deformed;
  deformPointsWithTimeCheck<pcl::PointXYZRGBA>(vertices,
                                               vertex_graph_map_deformed,
                                               old_vertices,
                                               stamps,
                                               prefix,
                                               vertex_positions_.at(prefix),
                                               vertex_stamps_.at(prefix),
                                               optimized_values,
                                               k,
                                               tol_t,
                                               indices_ptr);
  if (vertex_graph_map) {
    if (start_idx == 0) {
      *vertex_graph_map = vertex_graph_map_deformed;
    } else {
      vertex_graph_map->resize(vertices.size());
      for (size_t i = 0; i < indices_ptr->size(); i++) {
        vertex_graph_map->at(indices_ptr->at(i)) = vertex_graph_map_deformed.at(i);
      }
    }
  }

  // TODO(nathan) consider making these mutable to to make this method const
  last_calculated_vertices_[prefix] = vertices;
  recalculate_vertices_ = false;
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
}

void DeformationGraph::update() {
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

void fillDeformationGraphMarkers(const DeformationGraph& graph,
                                 const ros::Time& stamp,
                                 visualization_msgs::Marker& mesh_mesh_viz,
                                 visualization_msgs::Marker& pose_mesh_viz) {
  // First get the latest estimates and factors
  const auto& graph_values = graph.getGtsamValues();
  const auto& graph_factors = graph.getGtsamFactors();

  // header for the mesh to mesh edges
  mesh_mesh_viz.header.frame_id = "world";
  mesh_mesh_viz.header.stamp = stamp;
  mesh_mesh_viz.id = 0;
  mesh_mesh_viz.action = visualization_msgs::Marker::ADD;
  mesh_mesh_viz.type = visualization_msgs::Marker::LINE_LIST;
  mesh_mesh_viz.scale.x = 0.02;

  // header for the pose to mesh edges
  pose_mesh_viz.header.frame_id = "world";
  pose_mesh_viz.header.stamp = stamp;
  pose_mesh_viz.id = 0;
  pose_mesh_viz.action = visualization_msgs::Marker::ADD;
  pose_mesh_viz.type = visualization_msgs::Marker::LINE_LIST;
  pose_mesh_viz.scale.x = 0.02;

  for (auto factor : graph_factors) {
    // Only interested in edges here
    if (factor->keys().size() != 2) {
      continue;
    }

    const gtsam::Symbol& front = factor->front();
    const gtsam::Symbol& back = factor->back();

    const bool front_is_pose_vertex =
        (robot_prefix_to_id.find(front.chr()) != robot_prefix_to_id.end());
    const bool back_is_pose_vertex =
        (robot_prefix_to_id.find(back.chr()) != robot_prefix_to_id.end());

    // Three types: pose-to-pose, pose-to-mesh, mesh-to-mesh
    // color accordingly
    std_msgs::ColorRGBA color;
    if (front_is_pose_vertex && back_is_pose_vertex) {
      // pose-to-pose
      continue;
    }

    if (!front_is_pose_vertex && !back_is_pose_vertex) {
      // mesh-to-mesh
      mesh_mesh_viz.points.push_back(
          GtsamToRos(graph_values.at<gtsam::Pose3>(front)).position);
      mesh_mesh_viz.points.push_back(
          GtsamToRos(graph_values.at<gtsam::Pose3>(back)).position);
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 0.8;
      mesh_mesh_viz.colors.push_back(color);
      mesh_mesh_viz.colors.push_back(color);
    } else {
      // pose-to-mesh
      pose_mesh_viz.points.push_back(
          GtsamToRos(graph_values.at<gtsam::Pose3>(front)).position);
      pose_mesh_viz.points.push_back(
          GtsamToRos(graph_values.at<gtsam::Pose3>(back)).position);
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.2;
      color.a = 0.3;
      pose_mesh_viz.colors.push_back(color);
      pose_mesh_viz.colors.push_back(color);
    }
  }
}

}  // namespace kimera_pgmo
