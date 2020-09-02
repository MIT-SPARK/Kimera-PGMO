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
    : pgo_(nullptr), recalculate_vertices_(false) {}
DeformationGraph::~DeformationGraph() {}

bool DeformationGraph::initialize(double pgo_trans_threshold,
                                  double pgo_rot_threshold) {
  // Initialize pgo_:
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(
      pgo_trans_threshold, pgo_rot_threshold, KimeraRPGO::Verbosity::QUIET);
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(pgo_params));
  return true;
}

void DeformationGraph::updateMesh(
    const pcl::PointCloud<pcl::PointXYZRGBA>& new_vertices,
    const std::vector<pcl::Vertices> new_surfaces) {
  // Add new points to vertices
  size_t start_of_new_idx = vertex_positions_.size();
  for (auto p : new_vertices.points) {
    vertex_positions_.push_back(gtsam::Point3(p.x, p.y, p.z));
    vertices_.push_back(p);
  }

  // Create vector of new indices
  Vertices new_indices(new_vertices.size());
  std::iota(std::begin(new_indices), std::end(new_indices), start_of_new_idx);

  // Add to graph
  std::vector<Edge> new_edges =
      graph_.addPointsAndSurfaces(new_indices, new_surfaces);

  // Update consistency factors with new vertices and new edges
  updateConsistencyFactors(new_indices, new_edges);
}

void DeformationGraph::updateConsistencyFactors(
    const Vertices& new_vertices,
    const std::vector<Edge>& new_edges) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  // add new vertices to values
  for (Vertex v : new_vertices) {
    gtsam::Pose3 v_pose;
    gtsam::Symbol v_symb('v', v);
    v_pose = gtsam::Pose3(gtsam::Rot3(), vertex_positions_.at(v));
    new_values.insert(v_symb, v_pose);
  }

  // build the connections factors
  for (Edge e : new_edges) {
    Vertex from = e.first;
    Vertex to = e.second;
    gtsam::Pose3 from_pose;
    gtsam::Point3 to_point;
    gtsam::Symbol from_symb('v', from);
    gtsam::Symbol to_symb('v', to);
    from_pose = gtsam::Pose3(gtsam::Rot3(), vertex_positions_.at(from));

    to_point = vertex_positions_.at(to);

    // Define noise. Hardcoded for now
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(3, 1e-3);
    // Create deformation edge factor
    DeformationEdgeFactor new_edge(
        from_symb, to_symb, from_pose, to_point, noise);
    consistency_factors_.add(new_edge);
    new_factors.add(new_edge);
  }
  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
}

void DeformationGraph::addNodeValence(const size_t& i,
                                      const Vertices& valences) {
  gtsam::NonlinearFactorGraph new_factors;
  gtsam::Values new_values;

  pg_connections_.at(i) = valences;
  // Add the consistency factors
  for (Vertex v : valences) {
    gtsam::Symbol node('n', i);
    gtsam::Symbol vertex('v', v);
    gtsam::Pose3 node_pose = pg_initial_poses_.at(i);
    gtsam::Pose3 vertex_pose =
        gtsam::Pose3(gtsam::Rot3(), vertex_positions_.at(v));

    // Define noise. Hardcoded for now
    static const gtsam::SharedNoiseModel& noise =
        gtsam::noiseModel::Isotropic::Variance(3, 1e-3);
    // Create deformation edge factor
    DeformationEdgeFactor new_edge_1(
        node, vertex, node_pose, vertex_pose.translation(), noise);
    DeformationEdgeFactor new_edge_2(
        vertex, node, vertex_pose, node_pose.translation(), noise);
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
                                      const geometry_msgs::Pose& pose) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  // noise for measurement
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-10);

  gtsam::Symbol v_symb('v', v);
  gtsam::Pose3 meas = RosToGtsam(pose);
  gtsam::PriorFactor<gtsam::Pose3> absolute_meas(v_symb, meas, noise);
  prior_factors_.add(absolute_meas);
  new_factors.add(absolute_meas);

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  recalculate_vertices_ = true;
}

void DeformationGraph::addNodeMeasurement(const size_t& node_number,
                                          const gtsam::Pose3 delta_pose) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  gtsam::Symbol node_symb = gtsam::Symbol('n', node_number);
  Vertex node = node_symb.key();
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  gtsam::PriorFactor<gtsam::Pose3> measurement(node, delta_pose, noise);
  prior_factors_.add(measurement);
  new_factors.add(measurement);

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  recalculate_vertices_ = true;
}

void DeformationGraph::addNewBetween(const size_t& from,
                                     const size_t& to,
                                     const gtsam::Pose3& meas,
                                     const gtsam::Pose3& initial_pose) {
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  if (from >= pg_initial_poses_.size()) {
    ROS_ERROR(
        "DeformationGraph: when adding new between from key should already "
        "exist.");
    return;
  }

  gtsam::Symbol from_symb('n', from);
  gtsam::Symbol to_symb('n', to);

  if (to > pg_initial_poses_.size()) {
    ROS_ERROR("DeformationGraph: skipping keys in addNewBetween.");
    return;
  } else if (to == pg_initial_poses_.size()) {
    // new node
    // For now push empty valence, valences will be populated when updated
    Vertices valences;
    pg_initial_poses_.push_back(initial_pose);
    pg_connections_.push_back(valences);

    new_values.insert(to_symb, initial_pose);
  }

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  pg_factors_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(from_symb, to_symb, meas, noise));
  new_factors.add(
      gtsam::BetweenFactor<gtsam::Pose3>(from_symb, to_symb, meas, noise));

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();

  // if it's a loop closure factor
  if (to_symb != from_symb + 1) {
    ROS_INFO(
        "DeformationGraph: Added loop closure. Recalculating vertex "
        "positions.");
    recalculate_vertices_ = true;
  }
  return;
}

void DeformationGraph::initFirstNode(const gtsam::Pose3& initial_pose,
                                     bool add_prior) {
  // new node
  // For now push empty valence, valences will be populated when updated
  gtsam::Values new_values;
  gtsam::NonlinearFactorGraph new_factors;

  Vertices valences;
  pg_initial_poses_.push_back(initial_pose);
  pg_connections_.push_back(valences);

  gtsam::Symbol symb('n', 0);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  new_values.insert(symb, initial_pose);
  if (add_prior) {
    pg_factors_.add(
        gtsam::PriorFactor<gtsam::Pose3>(symb, initial_pose, noise));
    new_factors.add(
        gtsam::PriorFactor<gtsam::Pose3>(symb, initial_pose, noise));
  }

  pgo_->update(new_factors, new_values);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  return;
}

pcl::PolygonMesh DeformationGraph::deformMesh(
    const pcl::PolygonMesh& original_mesh,
    size_t k) {
  // Cannot deform if no nodes in the deformation graph
  if (vertices_.points.size() == 0) {
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

  for (size_t ii = start_idx; ii < original_vertices.points.size(); ii++) {
    pcl::PointXYZRGBA p = original_vertices.points[ii];
    // search for k + 1 nearest nodes
    std::vector<std::pair<Vertex, double>> nearest_nodes;
    gtsam::Point3 vi(p.x, p.y, p.z);
    for (size_t i = 0; i < vertices_.points.size(); i++) {
      pcl::PointXYZRGBA p_vertex = vertices_.points.at(i);
      double distance = (p.x - p_vertex.x) * (p.x - p_vertex.x) +
                        (p.y - p_vertex.y) * (p.y - p_vertex.y) +
                        (p.z - p_vertex.z) * (p.z - p_vertex.z);
      if (nearest_nodes.size() < k + 1 ||
          nearest_nodes.at(k).second > distance) {
        if (values_.exists(gtsam::Symbol('v', i))) {
          // make sure that the node in question has already been optimized
          nearest_nodes.push_back(std::pair<Vertex, double>(i, distance));
        }
      }
      // Sort according to distance
      auto compareFunc = [](std::pair<Vertex, double>& a,
                            std::pair<Vertex, double>& b) {
        return a.second < b.second;
      };
      std::sort(nearest_nodes.begin(), nearest_nodes.end(), compareFunc);

      // Keep length at k + 1
      if (nearest_nodes.size() > k + 1) {
        nearest_nodes.erase(nearest_nodes.begin() + k + 1);
      }
    }
    // Calculate new point location from k points
    gtsam::Point3 new_point(0, 0, 0);
    double d_max = std::sqrt(nearest_nodes.at(nearest_nodes.size() - 1).second);
    double weight_sum = 0;
    for (size_t j = 0; j < nearest_nodes.size() - 1; j++) {
      pcl::PointXYZRGBA p_g = vertices_.points.at(nearest_nodes[j].first);
      gtsam::Point3 gj(p_g.x, p_g.y, p_g.z);
      double weight = (1 - std::sqrt(nearest_nodes[j].second) / d_max);
      if (weight_sum == 0 && weight == 0) weight = 1;
      weight_sum = weight_sum + weight;
      gtsam::Pose3 node_transform =
          values_.at<gtsam::Pose3>(gtsam::Symbol('v', nearest_nodes[j].first));
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

std::vector<gtsam::Pose3> DeformationGraph::getOptimizedTrajectory() const {
  // return the optimized trajectory (pose graph)
  std::vector<gtsam::Pose3> optimized_traj;

  for (size_t i = 0; i < pg_initial_poses_.size(); i++) {
    gtsam::Symbol node('n', i);
    optimized_traj.push_back(values_.at<gtsam::Pose3>(node));
  }
  return optimized_traj;
}

}  // namespace kimera_pgmo