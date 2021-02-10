/**
 * @file   DeformationSolver.h
 * @brief  Deforms mesh based on lates optimized trajectory
 * @author Yun Chang
 */
#include <algorithm>
#include <cmath>

#include "kimera_pgmo/DeformationGraph.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using pcl::PolygonMesh;

namespace kimera_pgmo {

DeformationGraph::DeformationGraph()
    : pgo_(nullptr),
      recalculate_vertices_(false),
      vertices_(new pcl::PointCloud<pcl::PointXYZ>) {}
DeformationGraph::~DeformationGraph() {}

bool DeformationGraph::initialize(double pgo_trans_threshold,
                                  double pgo_rot_threshold) {
  // Initialize pgo_:
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(
      pgo_trans_threshold, pgo_rot_threshold, KimeraRPGO::Verbosity::UPDATE);
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(pgo_params));
  return true;
}

void DeformationGraph::updateMesh(
    const pcl::PointCloud<pcl::PointXYZRGBA>& new_vertices,
    const std::vector<pcl::Vertices> new_surfaces,
    const char& prefix) {
  // Check if prefix seen before
  if (num_vertices_.find(prefix) == num_vertices_.end())
    num_vertices_[prefix] = 0;
  // Add new points to vertices
  const size_t& start_of_new_idx = num_vertices_[prefix];
  // Create vector of new indices
  Vertices new_indices;
  for (auto p : new_vertices.points) {
    vertex_positions_.push_back(gtsam::Point3(p.x, p.y, p.z));
    vertex_prefixes_.push_back(prefix);
    vertices_->push_back(pcl::PointXYZ(p.x, p.y, p.z));
    new_indices.push_back(num_vertices_[prefix]);
    num_vertices_[prefix] = num_vertices_[prefix] + 1;
  }

  // Add to graph
  if (graph_.find(prefix) == graph_.end()) {
    graph_[prefix] = Graph();
  }
  const std::vector<Edge>& new_edges =
      graph_[prefix].addPointsAndSurfaces(new_indices, new_surfaces);

  // Update consistency factors with new vertices and new edges
  updateConsistencyFactors(new_indices, new_edges, prefix);
}

void DeformationGraph::updateConsistencyFactors(
    const Vertices& new_vertices,
    const std::vector<Edge>& new_edges,
    const char& prefix) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  // add new vertices to values
  for (Vertex v : new_vertices) {
    gtsam::Pose3 v_pose;
    gtsam::Symbol v_symb(prefix, v);
    v_pose = gtsam::Pose3(gtsam::Rot3(), vertex_positions_.at(v));
    new_values.insert(v_symb, v_pose);
  }

  // build the connections factors
  for (Edge e : new_edges) {
    Vertex from = e.first;
    Vertex to = e.second;
    const gtsam::Pose3 from_pose(gtsam::Rot3(), vertex_positions_.at(from));
    const gtsam::Point3& to_point = vertex_positions_.at(to);
    const gtsam::Symbol from_symb(prefix, from);
    const gtsam::Symbol to_symb(prefix, to);

    // Define noise. Hardcoded for now
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(3, 1e-3);
    // Create deformation edge factor
    const DeformationEdgeFactor new_edge(
        from_symb, to_symb, from_pose, to_point, noise);
    consistency_factors_.add(new_edge);
    new_factors.add(new_edge);
  }
  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
}

void DeformationGraph::addNodeValence(const gtsam::Key& key,
                                      const Vertices& valences,
                                      const char& valence_prefix) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  // Add the consistency factors
  for (Vertex v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);
    const gtsam::Pose3& node_pose = pg_initial_poses_[prefix].at(idx);
    const gtsam::Pose3 vertex_pose(gtsam::Rot3(), vertex_positions_.at(v));

    // Define noise. Hardcoded for now
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(3, 1e-3);
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
  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
}

void DeformationGraph::addMeasurement(const Vertex& v,
                                      const geometry_msgs::Pose& pose,
                                      const char& prefix) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  // noise for measurement
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-10);

  gtsam::Symbol v_symb(prefix, v);
  gtsam::Pose3 meas = RosToGtsam(pose);
  gtsam::PriorFactor<gtsam::Pose3> absolute_meas(v_symb, meas, noise);
  new_factors.add(absolute_meas);

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  recalculate_vertices_ = true;
}

void DeformationGraph::addNodeMeasurement(const gtsam::Key& key,
                                          const gtsam::Pose3 pose) {
  // TODO: consider consolidating with addNodeMeasurements
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  gtsam::PriorFactor<gtsam::Pose3> measurement(key, pose, noise);
  new_factors.add(measurement);

  if (!values_.exists(key)) {
    ROS_WARN(
        "DeformationGraph: adding node measurement to a node %s not previously "
        "seen before. ",
        gtsam::DefaultKeyFormatter(key));
    new_values.insert(key, pose);
  }

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  recalculate_vertices_ = true;
}

void DeformationGraph::addNodeMeasurements(
    const std::vector<std::pair<gtsam::Key, gtsam::Pose3>>& measurements) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  for (auto keyed_pose : measurements) {
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
    gtsam::PriorFactor<gtsam::Pose3> measurement(
        keyed_pose.first, keyed_pose.second, noise);
    new_factors.add(measurement);

    if (!values_.exists(keyed_pose.first)) {
      ROS_WARN(
          "DeformationGraph: adding node measurement to a node %s not "
          "previously seen before. ",
          gtsam::DefaultKeyFormatter(keyed_pose.first));
      new_values.insert(keyed_pose.first, keyed_pose.second);
    }
  }

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  recalculate_vertices_ = true;
}

void DeformationGraph::addNewBetween(const gtsam::Key& key_from,
                                     const gtsam::Key& key_to,
                                     const gtsam::Pose3& meas,
                                     const gtsam::Pose3& initial_pose) {
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

    new_values.insert(key_to, initial_pose);
  }

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  new_factors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();

  // if it's a loop closure factor
  if (key_to != key_from + 1) {
    ROS_INFO(
        "DeformationGraph: Added loop closure. Recalculating vertex "
        "positions.");
    recalculate_vertices_ = true;
  }
  return;
}

void DeformationGraph::addNewNode(const gtsam::Key& key,
                                  const gtsam::Pose3& initial_pose,
                                  bool add_prior) {
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

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  new_values.insert(key, initial_pose);
  if (add_prior) {
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, initial_pose, noise));
  }

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
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
    const char& prefix,
    size_t k) {
  // Cannot deform if no nodes in the deformation graph
  if (vertices_->points.size() == 0) {
    ROS_DEBUG("Deformable mesh empty. No deformation. ");
    return original_mesh;
  }
  // extract original vertices
  pcl::PointCloud<pcl::PointXYZRGBA> original_vertices;
  pcl::fromPCLPointCloud2(original_mesh.cloud, original_vertices);

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  // iterate through original vertices to create new vertices
  size_t start_idx = 0;
  if (!recalculate_vertices_) {
    start_idx = last_calculated_vertices_.points.size();
    new_vertices = last_calculated_vertices_;
  }

  // Build Octree
  Octree::Ptr search_octree(new Octree(1.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  search_octree->setInputCloud(search_cloud);
  for (size_t i = 0; i < vertex_prefixes_.size(); i++) {
    if (vertex_prefixes_[i] == prefix) {
      search_cloud->push_back(vertices_->points[i]);
      search_octree->addPointFromCloud(search_cloud->points.size() - 1,
                                       nullptr);
    }
  }

  for (size_t ii = start_idx; ii < original_vertices.points.size(); ii++) {
    const pcl::PointXYZRGBA& p = original_vertices.points[ii];
    // search for k + 1 nearest nodes
    std::vector<std::pair<Vertex, double>> nearest_nodes;
    pcl::PointXYZ p_xyz(p.x, p.y, p.z);
    gtsam::Point3 vi(p.x, p.y, p.z);
    // Query octree
    std::vector<int> nearest_nodes_index;
    std::vector<float> nearest_nodes_sq_dist;
    search_octree->nearestKSearch(
        p_xyz, k + 1, nearest_nodes_index, nearest_nodes_sq_dist);

    // Calculate new point location from k points
    gtsam::Point3 new_point(0, 0, 0);
    double d_max =
        std::sqrt(nearest_nodes_sq_dist[nearest_nodes_index.size() - 1]);
    double weight_sum = 0;
    for (size_t j = 0; j < nearest_nodes_index.size() - 1; j++) {
      const pcl::PointXYZ& p_g =
          search_cloud->points.at(nearest_nodes_index[j]);
      gtsam::Point3 gj(p_g.x, p_g.y, p_g.z);
      double weight = (1 - std::sqrt(nearest_nodes_sq_dist[j]) / d_max);
      if (weight_sum == 0 && weight == 0) weight = 1;
      weight_sum = weight_sum + weight;
      gtsam::Pose3 node_transform = values_.at<gtsam::Pose3>(
          gtsam::Symbol(prefix, nearest_nodes_index[j]));
      gtsam::Point3 add = node_transform.rotation().rotate(vi - gj) +
                          node_transform.translation();
      new_point = new_point + weight * add;
    }
    // Add back to new_vertices
    pcl::PointXYZRGBA new_pcl_point;
    new_pcl_point.x = new_point.x() / weight_sum;
    new_pcl_point.y = new_point.y() / weight_sum;
    new_pcl_point.z = new_point.z() / weight_sum;
    new_pcl_point.r = p.r;
    new_pcl_point.g = p.g;
    new_pcl_point.b = p.b;
    new_pcl_point.a = p.a;
    new_vertices.points.push_back(new_pcl_point);
  }

  // With new vertices, construct new polygon mesh
  pcl::PolygonMesh new_mesh;
  new_mesh.polygons = original_mesh.polygons;
  pcl::toPCLPointCloud2(new_vertices, new_mesh.cloud);
  last_calculated_vertices_ = new_vertices;
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

}  // namespace kimera_pgmo