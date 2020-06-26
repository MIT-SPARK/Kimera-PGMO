/**
 * @file   DeformationSolver.h
 * @brief  Deforms mesh based on lates optimized trajectory
 * @author Yun Chang
 */
#include <algorithm>
#include <cmath>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/DeformationGraph.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/console.h>

#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using pcl::PolygonMesh;

namespace mesher_mapper {

DeformationGraph::DeformationGraph() {}
DeformationGraph::~DeformationGraph() {}

void DeformationGraph::update() {
  // Here we assume that the mesh since creation has grown
  // Essentially every new mesh recieved contains the old mesh
  graph_ = Graph();
  graph_.createFromPclMeshBidirection(mesh_structure_);
  // store points position
  pcl::fromPCLPointCloud2(mesh_structure_.cloud, vertices_);
  for (size_t i = 0; i < vertices_.points.size(); i++) {
    vertex_positions_[i] = gtsam::Point3(
        vertices_.points[i].x, vertices_.points[i].y, vertices_.points[i].z);
  }

  // Add the non mesh nodes
  addNonMeshNodesToGraph();

  // reset consistency factors
  resetConsistencyFactors();
}

void DeformationGraph::resetConsistencyFactors() {
  values_ = gtsam::Values();
  consistency_factors_ = gtsam::NonlinearFactorGraph();
  // build the connections factors
  // This consist of all the factors used
  // in optimization without the factors
  // related to user input
  Vertices vertices = graph_.getVertices();
  for (Vertex v : vertices) {
    // Check if vertex has a prefix (ie if is a pose graph node)
    gtsam::Pose3 v_pose;
    gtsam::Symbol v_symb(v);
    if (v_symb.chr() == 'n') {
      size_t node_number = v_symb.index();
      v_pose = pg_initial_poses_.at(node_number);
    } else {
      v_pose = gtsam::Pose3(gtsam::Rot3(), vertex_positions_.at(v));
    }
    values_.insert(v, v_pose);

    try {
      for (Vertex valence : graph_.getValence(v)) {
        gtsam::Point3 valence_pos = vertex_positions_[valence];
        // Define noise. Hardcoded for now
        static const gtsam::SharedNoiseModel& noise =
            gtsam::noiseModel::Isotropic::Variance(3, 1e-3);
        // Create deformation edge factor
        DeformationEdgeFactor new_edge(v, valence, v_pose, valence_pos, noise);
        consistency_factors_.add(new_edge);
      }
    } catch (const std::out_of_range& e) {
      ROS_DEBUG("No valence for node %d", v);
    }
  }
}

void DeformationGraph::addNonMeshNodesToGraph() {
  for (size_t i = 0; i < pg_vertices_.size(); i++) {
    vertices_.points.push_back(pg_vertices_[i]);
    gtsam::Symbol node_symb('n', i);
    Vertex node = node_symb.key();
    graph_.addVertex(node);
    // add vertex position to map
    vertex_positions_[node] =
        gtsam::Point3(pg_vertices_[i].x, pg_vertices_[i].y, pg_vertices_[i].z);
    for (Vertex valence : pg_connections_[i]) {
      graph_.addEdge(Edge(node, valence));
      graph_.addEdge(Edge(valence, node));
    }
  }
}

void DeformationGraph::addNode(const pcl::PointXYZ& position,
                               Vertices valences,
                               bool connect_to_previous) {
  // if it is a trajectory node, add connection to previous added node
  if (connect_to_previous && pg_vertices_.size() > 0) {
    size_t last_index = pg_vertices_.size() - 1;
    valences.push_back(gtsam::Symbol('n', last_index).key());
  }
  gtsam::Pose3 init_pose(gtsam::Rot3(),
                         gtsam::Point3(position.x, position.y, position.z));
  pg_initial_poses_.push_back(init_pose);
  pg_vertices_.push_back(position);
  pg_connections_.push_back(valences);
}

void DeformationGraph::updateNodeValence(size_t i,
                                         Vertices valences,
                                         bool connect_to_previous) {
  // if it is a trajectory node, add connection to previous added node
  if (connect_to_previous && i > 0) {
    size_t last_index = i - 1;
    valences.push_back(gtsam::Symbol('n', last_index).key());
  }
  pg_connections_.at(i) = valences;
}

void DeformationGraph::addMeasurement(const Vertex& v,
                                      const geometry_msgs::Pose& pose) {
  // noise for measurement
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-10);

  gtsam::Pose3 meas = RosToGtsam(pose);
  gtsam::PriorFactor<gtsam::Pose3> absolute_meas(v, meas, noise);
  prior_factors_.add(absolute_meas);
}

void DeformationGraph::addNodeMeasurement(const size_t& node_number,
                                          const gtsam::Pose3 delta_pose) {
  gtsam::Symbol node_symb = gtsam::Symbol('n', node_number);
  Vertex node = node_symb.key();
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  gtsam::PriorFactor<gtsam::Pose3> measurement(node, delta_pose, noise);
  prior_factors_.add(measurement);
}

void DeformationGraph::optimize() {
  gtsam::NonlinearFactorGraph factors_to_optimize;
  factors_to_optimize.add(consistency_factors_);
  factors_to_optimize.add(prior_factors_);
  // optimize
  gtsam::LevenbergMarquardtParams params;
  params.diagonalDamping = true;
  params.setVerbosityLM("SUMMARY");
  values_ =
      gtsam::LevenbergMarquardtOptimizer(factors_to_optimize, values_, params)
          .optimize();
}

pcl::PolygonMesh DeformationGraph::deformMesh(
    const pcl::PolygonMesh& original_mesh,
    size_t k) const {
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
  // TODO (Yun) make this part faster
  for (pcl::PointXYZRGBA p : original_vertices.points) {
    // search for k + 1 nearest nodes
    std::vector<std::pair<Vertex, double>> nearest_nodes;
    gtsam::Point3 vi(p.x, p.y, p.z);
    for (size_t i = 0; i < vertices_.points.size(); i++) {
      pcl::PointXYZ p_vertex = vertices_.points.at(i);
      double distance = (p.x - p_vertex.x) * (p.x - p_vertex.x) +
                        (p.y - p_vertex.y) * (p.y - p_vertex.y) +
                        (p.z - p_vertex.z) * (p.z - p_vertex.z);
      if (nearest_nodes.size() < k + 1 ||
          nearest_nodes.at(k).second > distance) {
        if (values_.exists(i)) {
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
      pcl::PointXYZ p_g = vertices_.points.at(nearest_nodes[j].first);
      gtsam::Point3 gj(p_g.x, p_g.y, p_g.z);
      double weight = (1 - std::sqrt(nearest_nodes[j].second) / d_max);
      if (weight_sum == 0 && weight == 0) weight = 1;
      weight_sum = weight_sum + weight;
      gtsam::Pose3 node_transform =
          values_.at<gtsam::Pose3>(nearest_nodes[j].first);
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

  return new_mesh;
}

void DeformationGraph::addNewBetween(const size_t& from,
                                     const size_t& to,
                                     const gtsam::Pose3& meas,
                                     const gtsam::Pose3& initial_pose) {
  if (from >= pg_vertices_.size()) {
    ROS_ERROR(
        "DeformationGraph: when adding new between from key should already "
        "exist.");
    return;
  }
  if (to > pg_vertices_.size()) {
    ROS_ERROR("DeformationGraph: skipping keys in addNewBetween.");
    return;
  } else if (to == pg_vertices_.size()) {
    // new node
    // For now push empty valence, valences will be populated when updated
    Vertices valences;
    pg_vertices_.push_back(
        pcl::PointXYZ(initial_pose.x(), initial_pose.y(), initial_pose.z()));
    pg_initial_poses_.push_back(initial_pose);
    pg_connections_.push_back(valences);
  }

  gtsam::Symbol from_symb('n', from);
  gtsam::Symbol to_symb('n', to);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  pg_factors_.add(
      gtsam::BetweenFactor<gtsam::Pose3>(from_symb, to_symb, meas, noise));
  return;
}

void DeformationGraph::initFirstNode(const gtsam::Pose3& initial_pose) {
  // new node
  // For now push empty valence, valences will be populated when updated
  Vertices valences;
  pg_vertices_.push_back(
      pcl::PointXYZ(initial_pose.x(), initial_pose.y(), initial_pose.z()));
  pg_initial_poses_.push_back(initial_pose);
  pg_connections_.push_back(valences);

  gtsam::Symbol symb('n', 0);
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-15);
  pg_factors_.add(gtsam::PriorFactor<gtsam::Pose3>(symb, initial_pose, noise));
  return;
}

}  // namespace mesher_mapper