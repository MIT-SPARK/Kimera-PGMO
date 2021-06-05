/**
 * @file   DeformationSolver.h
 * @brief  Deforms mesh based on lates optimized trajectory
 * @author Yun Chang
 */
#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <algorithm>
#include <cmath>

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
    : pgo_(nullptr),
      recalculate_vertices_(false),
      vertices_(new pcl::PointCloud<pcl::PointXYZ>),
      do_not_optimize_(false) {}
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

void DeformationGraph::addNodeValence(const gtsam::Key& key,
                                      const Vertices& valences,
                                      const char& valence_prefix,
                                      bool optimize) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;
  const char& prefix = gtsam::Symbol(key).chr();
  const size_t& idx = gtsam::Symbol(key).index();
  // Add the consistency factors
  for (Vertex v : valences) {
    const gtsam::Symbol vertex(valence_prefix, v);
    const gtsam::Pose3& node_pose = pg_initial_poses_[prefix].at(idx);
    const gtsam::Pose3 vertex_pose(gtsam::Rot3(),
                                   vertex_positions_[valence_prefix].at(v));

    // Define noise. Hardcoded for now
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(3, 1e-4);
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

  pgo_->update(new_factors, new_values, !do_not_optimize_);
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
      gtsam::noiseModel::Isotropic::Variance(6, 1e-4);
  gtsam::PriorFactor<gtsam::Pose3> measurement(key, pose, noise);
  new_factors.add(measurement);

  if (!values_.exists(key)) {
    ROS_WARN(
        "DeformationGraph: adding node measurement to a node %s not previously "
        "seen before. ",
        gtsam::DefaultKeyFormatter(key));
    new_values.insert(key, pose);
  }

  pgo_->update(new_factors, new_values, !do_not_optimize_);
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
        gtsam::noiseModel::Isotropic::Variance(6, 1e-4);
    gtsam::PriorFactor<gtsam::Pose3> measurement(
        keyed_pose.first, keyed_pose.second, noise);
    new_factors.add(measurement);

    if (!values_.exists(keyed_pose.first)) {
      ROS_ERROR(
          "DeformationGraph: adding node measurement to a node %s not "
          "previously seen before. ",
          gtsam::DefaultKeyFormatter(keyed_pose.first));
    }
  }

  pgo_->update(new_factors, new_values, !do_not_optimize_);
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

    gtsam::Pose3 initial_estimate = initial_pose;
    if (to_idx > 0) {
      initial_estimate =
          pg_initial_poses_[to_prefix].at(to_idx - 1).compose(meas);
    }
    new_values.insert(key_to, initial_estimate);
  }

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-4);
  new_factors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(key_from, key_to, meas, noise));

  pgo_->update(new_factors, new_values, !do_not_optimize_);
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

void DeformationGraph::addNewMeshEdgesAndNodes(
    const std::vector<std::pair<gtsam::Key, gtsam::Key>>& mesh_edges,
    const gtsam::Values& mesh_nodes,
    bool optimize) {
  // New mesh edge factors
  gtsam::NonlinearFactorGraph new_mesh_factors;
  // New mesh node factors
  gtsam::Values new_mesh_nodes;
  // Define noise. Hardcoded for now
  static const gtsam::SharedNoiseModel& edge_noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-4);
  // Iterate and add the new edges
  for (auto e : mesh_edges) {
    const gtsam::Key& from = e.first;
    const gtsam::Key& to = e.second;
    const gtsam::Pose3& pose_from = mesh_nodes.at<gtsam::Pose3>(from);
    const gtsam::Pose3& pose_to = mesh_nodes.at<gtsam::Pose3>(to);
    // Create new edge as deformation edge factor
    const DeformationEdgeFactor new_edge(
        from, to, pose_from, pose_to.translation(), edge_noise);
    new_mesh_factors.add(new_edge);
    consistency_factors_.add(new_edge);
  }
  // Iterate and add the new mesh nodes not yet in graph
  // Note that the keys are in increasing order by construction from gtsam
  for (auto k : mesh_nodes.keys()) {
    char node_prefix = gtsam::Symbol(k).chr();
    size_t node_idx = gtsam::Symbol(k).index();
    const gtsam::Pose3& node_pose = mesh_nodes.at<gtsam::Pose3>(k);
    try {
      if (node_idx == vertex_positions_.at(node_prefix).size()) {
        // Only add nodes that has not previously been added
        vertex_positions_[node_prefix].push_back(node_pose.translation());
        vertices_->push_back(
            GtsamToPcl<pcl::PointXYZ>(node_pose.translation()));
        new_mesh_nodes.insert(k, node_pose);
      } else if (node_idx > vertex_positions_.at(node_prefix).size()) {
        ROS_ERROR_STREAM(
            "Adding new mesh edges and nodes: node index does not match index "
            "in vertex position vector. Likely to have dropped packets from "
            "frontend. "
            << node_idx << " vs. " << vertex_positions_.at(node_prefix).size());
        exit(EXIT_FAILURE);
      }
    } catch (const std::out_of_range& e) {
      ROS_INFO("New prefix detected when adding new mesh edges and nodes. ");
      vertex_positions_[node_prefix] = std::vector<gtsam::Point3>{};
      vertex_positions_[node_prefix].push_back(node_pose.translation());
      vertices_->push_back(GtsamToPcl<pcl::PointXYZ>(node_pose.translation()));
      new_mesh_nodes.insert(k, node_pose);
    }
  }

  // Update rpgo
  if (do_not_optimize_) optimize = false;
  pgo_->update(new_mesh_factors, new_mesh_nodes, optimize);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
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
      gtsam::noiseModel::Isotropic::Variance(6, 1e-4);
  new_values.insert(key, initial_pose);
  if (add_prior) {
    new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(key, initial_pose, noise));
  }

  pgo_->update(new_factors, new_values, !do_not_optimize_);
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
  return deformMesh(original_mesh, prefix, values_, k);
}

pcl::PolygonMesh DeformationGraph::deformMesh(
    const pcl::PolygonMesh& original_mesh,
    const char& prefix,
    const gtsam::Values& optimized_values,
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
  if (!recalculate_vertices_ && last_calculated_vertices_.find(prefix) !=
                                    last_calculated_vertices_.end()) {
    start_idx = last_calculated_vertices_[prefix].points.size();
    new_vertices = last_calculated_vertices_[prefix];
  }

  // Build Octree
  Octree::Ptr search_octree(new Octree(1.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  search_octree->setInputCloud(search_cloud);
  for (auto v : optimized_values.keys()) {
    gtsam::Symbol v_key = gtsam::Symbol(v);
    if (v_key.chr() == prefix) {
      // Gtsam Keylist should be ordered
      gtsam::Point3 position = vertex_positions_[prefix].at(v_key.index());
      search_cloud->push_back(
          pcl::PointXYZ(position.x(), position.y(), position.z()));
      search_octree->addPointFromCloud(search_cloud->points.size() - 1,
                                       nullptr);
    }
  }

  if (search_cloud->size() < k) {
    ROS_WARN("Not enough vertices to deform mesh. ");
    return original_mesh;
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
      gtsam::Pose3 node_transform = optimized_values.at<gtsam::Pose3>(
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

}  // namespace kimera_pgmo