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
      force_recalculate_(true),
      do_not_optimize_(false) {}
DeformationGraph::~DeformationGraph() {}

bool DeformationGraph::initialize(
    const KimeraRPGO::RobustSolverParams& params) {
  // Initialize RPGO
  pgo_params_ = params;
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(params));
  return true;
}

void DeformationGraph::addNodeValence(const gtsam::Key& key,
                                      const Vertices& valences,
                                      const char& valence_prefix,
                                      double variance,
                                      bool optimize) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  // Add the consistency factors
  for (Vertex v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);
    if (!values_.exists(vertex)) continue;
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
    new_factors.add(new_edge_1);
    new_factors.add(new_edge_2);
  }
  pgo_->update(new_factors, new_values, optimize);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  gnc_weights_ = pgo_->getGncWeights();
}

void DeformationGraph::addTempNodeValence(const gtsam::Key& key,
                                          const Vertices& valences,
                                          const char& valence_prefix,
                                          double variance,
                                          bool optimize) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  // Add the consistency factors
  for (Vertex v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);
    if (!values_.exists(vertex)) continue;

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
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

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
  new_factors.add(absolute_meas);

  pgo_->update(new_factors, new_values, !do_not_optimize_);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  gnc_weights_ = pgo_->getGncWeights();
  recalculate_vertices_ = true;
}

void DeformationGraph::addNodeMeasurements(
    const std::vector<std::pair<gtsam::Key, gtsam::Pose3>>& measurements,
    double variance) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  for (auto keyed_pose : measurements) {
    if (!values_.exists(keyed_pose.first)) {
      ROS_ERROR(
          "DeformationGraph: adding node measurement to a node %s not "
          "previously seen before. ",
          gtsam::DefaultKeyFormatter(keyed_pose.first));
      new_values.insert(keyed_pose.first, keyed_pose.second);
    }
    gtsam::Vector6 variances;
    variances.head<3>().setConstant(1e-02 * variance);
    variances.tail<3>().setConstant(variance);
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Diagonal::Variances(variances);
    gtsam::PriorFactor<gtsam::Pose3> measurement(
        keyed_pose.first, keyed_pose.second, noise);
    new_factors.add(measurement);
  }

  pgo_->update(new_factors, new_values, !do_not_optimize_);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  gnc_weights_ = pgo_->getGncWeights();
  recalculate_vertices_ = true;
}

void DeformationGraph::addNewBetween(const gtsam::Key& key_from,
                                     const gtsam::Key& key_to,
                                     const gtsam::Pose3& meas,
                                     const gtsam::Pose3& initial_pose,
                                     double variance) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;
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
      } else if (new_values.exists(key_from)) {
        initial_estimate = new_values.at<gtsam::Pose3>(key_from).compose(meas);
      }
    }
    new_values.insert(key_to, initial_estimate);
  }

  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);
  new_factors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  pgo_->update(new_factors, new_values, !do_not_optimize_);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  gnc_weights_ = pgo_->getGncWeights();

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
  if (!values_.exists(key_from) && !temp_values_.exists(key_from)) {
    ROS_ERROR("Key does not exist when adding temporary between factor. ");
    return;
  }

  if (!values_.exists(key_to) && !temp_values_.exists(key_to)) {
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
  new_factors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  pgo_->updateTempFactorsValues(new_factors, new_values);
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  temp_values_ = pgo_->getTempValues();
  return;
}

// TODO(yun) add unittest
void DeformationGraph::addNewTempEdges(
    const pose_graph_tools::PoseGraph& edges,
    double variance) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  gtsam::Vector6 variances;
  variances.head<3>().setConstant(1e-02 * variance);
  variances.tail<3>().setConstant(variance);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Diagonal::Variances(variances);

  for (const auto& e : edges.edges) {
    if (!values_.exists(e.key_from) && !temp_values_.exists(e.key_from)) {
      ROS_ERROR("Key does not exist when adding temporary between factor. ");
      continue;
    }

    if (!values_.exists(e.key_to) && !temp_values_.exists(e.key_to)) {
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
    const ros::Time& stamp,
    std::vector<size_t>* added_indices,
    double variance,
    bool optimize) {
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
          vertex_stamps_[node_prefix].push_back(stamp);
        }
      }
      if (node_idx == vertex_positions_.at(node_prefix).size()) {
        // Only add nodes that has not previously been added
        vertex_positions_[node_prefix].push_back(node_pose.translation());
        vertex_stamps_[node_prefix].push_back(stamp);
        new_mesh_nodes.insert(k, node_pose);
        added_indices->push_back(node_idx);
      }
    } catch (const std::out_of_range& e) {
      if (verbose_) {
        ROS_INFO_STREAM("New prefix " << node_prefix <<
                        " detected when adding new mesh edges and nodes. ");
      }
      vertex_positions_[node_prefix] = std::vector<gtsam::Point3>{};
      vertex_stamps_[node_prefix] = std::vector<ros::Time>{};
      vertex_positions_[node_prefix].push_back(node_pose.translation());
      vertex_stamps_[node_prefix].push_back(stamp);
      new_mesh_nodes.insert(k, node_pose);
      added_indices->push_back(node_idx);
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
    if ((!values_.exists(from) && !new_mesh_nodes.exists(from)) ||
        (!values_.exists(to) && !new_mesh_nodes.exists(to)))
      continue;
    const gtsam::Pose3& pose_from = mesh_nodes.at<gtsam::Pose3>(from);
    const gtsam::Pose3& pose_to = mesh_nodes.at<gtsam::Pose3>(to);
    // Create new edge as deformation edge factor
    const DeformationEdgeFactor new_edge(
        from, to, pose_from, pose_to.translation(), edge_noise);
    new_mesh_factors.add(new_edge);
    consistency_factors_.add(new_edge);
  }

  // Update rpgo
  if (do_not_optimize_) optimize = false;
  pgo_->update(new_mesh_factors, new_mesh_nodes, optimize);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  gnc_weights_ = pgo_->getGncWeights();
}

void DeformationGraph::addNewNode(const gtsam::Key& key,
                                  const gtsam::Pose3& initial_pose,
                                  bool add_prior,
                                  double prior_variance) {
  // new node
  // For now push empty valence, valences will be populated when updated
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

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
  new_values.insert(key, initial_pose);
  if (add_prior) {
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, initial_pose, noise));
  }

  pgo_->update(new_factors, new_values, !do_not_optimize_);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  gnc_weights_ = pgo_->getGncWeights();
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
    new_values.insert(keys[i], initial_poses[i]);
    if (add_prior) {
      new_factors.add(
          gtsam::PriorFactor<gtsam::Pose3>(keys[i], initial_poses[i], noise));
    }

    for (Vertex v : valences[i]) {
      const gtsam::Symbol vertex(valence_prefix, v);
      if (!values_.exists(vertex)) continue;

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

pcl::PolygonMesh DeformationGraph::deformMesh(
    const pcl::PolygonMesh& original_mesh,
    const std::vector<ros::Time>& stamps,
    const std::vector<int>& graph_indices,
    const char& prefix,
    size_t k,
    double tol_t) {
  return deformMesh(
      original_mesh, stamps, graph_indices, prefix, values_, k, tol_t);
}

pcl::PolygonMesh DeformationGraph::deformMesh(
    const pcl::PolygonMesh& original_mesh,
    const std::vector<ros::Time>& stamps,
    const std::vector<int>& graph_indices,
    const char& prefix,
    const gtsam::Values& optimized_values,
    size_t k,
    double tol_t) {
  // Cannot deform if no nodes in the deformation graph
  if (vertex_positions_.find(prefix) == vertex_positions_.end()) {
    ROS_DEBUG(
        "Deformation graph has no vertices associated with mesh prefix. No "
        "deformation. ");
    return original_mesh;
  }

  // extract original vertices
  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  pcl::fromPCLPointCloud2(original_mesh.cloud, new_vertices);

  // iterate through original vertices to create new vertices
  size_t start_idx = 0;
  if (!recalculate_vertices_ && last_calculated_vertices_.find(prefix) !=
                                    last_calculated_vertices_.end()) {
    ros::Time min_stamp =
        ros::Time(std::max(0.0, vertex_stamps_[prefix].back().toSec() - tol_t));
    auto bound = std::upper_bound(stamps.begin(), stamps.end(), min_stamp);
    start_idx = std::min(static_cast<size_t>(bound - stamps.begin()),
                         last_calculated_vertices_[prefix].size());
    for (size_t i = 0; i < start_idx; i++) {
      new_vertices.points[i] = last_calculated_vertices_[prefix].points[i];
    }
  } else {
    ROS_INFO(
        "DeformationGraph: Re-calculating all mesh vertices in deformMesh. ");
  }

  pcl::PointCloud<pcl::PointXYZRGBA> vertices_to_deform;
  std::vector<ros::Time> stamps_to_deform;
  std::vector<size_t> indices_to_deform;
  for (size_t i = start_idx; i < new_vertices.size(); i++) {
    const int& index = graph_indices.at(i);
    if (index < 0 || !optimized_values.exists(gtsam::Symbol(prefix, index))) {
      // Have to check here because sometimes interpolation happen before mesh
      // graph received TODO(yun) double check this
      vertices_to_deform.points.push_back(new_vertices.points[i]);
      stamps_to_deform.push_back(stamps.at(i));
      indices_to_deform.push_back(i);
    } else {
      const pcl::PointXYZRGBA& orig_p = new_vertices.points[i];
      gtsam::Pose3 node_transform =
          optimized_values.at<gtsam::Pose3>(gtsam::Symbol(prefix, index));
      gtsam::Point3 vi(orig_p.x, orig_p.y, orig_p.z);
      gtsam::Point3 gindex = vertex_positions_[prefix].at(index);
      gtsam::Point3 deformed_point =
          node_transform.rotation().rotate(vi - gindex) +
          node_transform.translation();
      pcl::PointXYZRGBA new_p = new_vertices.points[i];
      new_p.x = deformed_point.x();
      new_p.y = deformed_point.y();
      new_p.z = deformed_point.z();
      new_vertices.points[i] = new_p;
    }
  }

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices_to_deform =
      deformPointsWithTimeCheck<pcl::PointXYZRGBA>(vertices_to_deform,
                                                   stamps_to_deform,
                                                   prefix,
                                                   vertex_positions_[prefix],
                                                   vertex_stamps_[prefix],
                                                   optimized_values,
                                                   k,
                                                   tol_t);

  for (size_t i = 0; i < indices_to_deform.size(); i++) {
    new_vertices.points[indices_to_deform[i]] =
        new_vertices_to_deform.points[i];
  }

  // With new vertices, construct new polygon mesh
  pcl::PolygonMesh new_mesh;
  new_mesh.polygons = original_mesh.polygons;
  pcl::toPCLPointCloud2(new_vertices, new_mesh.cloud);
  last_calculated_vertices_[prefix] = new_vertices;
  recalculate_vertices_ = false;
  return new_mesh;
}

std::vector<gtsam::Pose3> DeformationGraph::getOptimizedTrajectory(
    char prefix) const {
  // return the optimized trajectory (pose graph)
  std::vector<gtsam::Pose3> optimized_traj;

  for (size_t i = 0; i < pg_initial_poses_.at(prefix).size(); i++) {
    gtsam::Symbol node(prefix, i);
    optimized_traj.push_back(values_.at<gtsam::Pose3>(node));
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

void DeformationGraph::getGtsamTempFactorsFiltered(
    gtsam::NonlinearFactorGraph* nfg,
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

void DeformationGraph::setParams(const KimeraRPGO::RobustSolverParams& params) {
  pgo_params_ = params;
  pgo_.reset(new KimeraRPGO::RobustSolver(pgo_params_));
}

}  // namespace kimera_pgmo
