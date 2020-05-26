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

#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/PriorFactor.h>

using pcl::PolygonMesh;

namespace mesher_mapper {

DeformationGraph::DeformationGraph() {}
DeformationGraph::~DeformationGraph() {}

void DeformationGraph::createFromMesh(const pcl::PolygonMesh& mesh) {
  // store mesh
  graph_.createFromPclMeshBidirection(mesh);
  // store points position
  pcl::fromPCLPointCloud2(mesh.cloud, vertices_);

  // build the connections factors
  // This consist of all the factors used
  // in optimization without the factors
  // related to user input
  Vertices vertices = graph_.getVertices();
  for (Vertex v : vertices) {
    values_.insert(v, gtsam::Pose3());
    pcl::PointXYZ p = vertices_.points[v];
    gtsam::Point3 v_pos(p.x, p.y, p.z);
    for (Vertex valence : graph_.getValence(v)) {
      pcl::PointXYZ pv = vertices_.points[valence];
      gtsam::Point3 valence_pos(pv.x, pv.y, pv.z);
      // Define noise. Hardcoded for now
      static const gtsam::SharedNoiseModel& noise =
          gtsam::noiseModel::Isotropic::Variance(3, 1e-3);
      // Create deformation edge factor
      DeformationEdgeFactor new_edge(v, valence, v_pos, valence_pos, noise);
      consistency_factors_.add(new_edge);
    }
  }
}

void DeformationGraph::addMesh(const pcl::PolygonMesh& mesh) {
  // Add new mesh (note does not check for repeated vertices and surfaces)
  Graph new_graph;
  new_graph.createFromPclMesh(mesh);

  pcl::PointCloud<pcl::PointXYZ> new_vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, new_vertices);

  graph_.combineGraph(new_graph);

  for (pcl::PointXYZ p : new_vertices.points) {
    vertices_.points.push_back(p);
  }
}

void DeformationGraph::addMeasurement(const Vertex& v,
                                      const geometry_msgs::Pose& pose) {
  // noise for loop closure
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 1e-3);

  gtsam::Pose3 meas = RosToGtsam(pose);
  gtsam::PriorFactor<gtsam::Pose3> absolute_meas(v, meas, noise);
  prior_factors_.add(absolute_meas);
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
  // extract original vertices
  pcl::PointCloud<pcl::PointXYZ> original_vertices;
  pcl::fromPCLPointCloud2(original_mesh.cloud, original_vertices);

  pcl::PointCloud<pcl::PointXYZ> new_vertices;
  // iterate through original vertices to create new vertices
  // TODO (Yun) make this part faster
  for (pcl::PointXYZ p : original_vertices.points) {
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
        nearest_nodes.push_back(std::pair<Vertex, double>(i, distance));
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
    double d_max = std::sqrt(nearest_nodes[nearest_nodes.size() - 1].second);
    double weight_sum = 0;
    for (size_t j = 0; j < nearest_nodes.size() - 1; j++) {
      pcl::PointXYZ p_g = vertices_.points.at(nearest_nodes[j].first);
      gtsam::Point3 gj(p_g.x, p_g.y, p_g.z);
      double weight = (1 - std::sqrt(nearest_nodes[j].second) / d_max);
      if (weight_sum == 0 && weight == 0) weight = 1;
      weight_sum = weight_sum + weight;
      gtsam::Pose3 node_transform =
          values_.at<gtsam::Pose3>(nearest_nodes[j].first);
      gtsam::Point3 add = node_transform.rotation().rotate(vi - gj) + gj +
                          node_transform.translation();
      new_point = new_point + weight * add;
    }
    // Add back to new_vertices
    new_vertices.points.push_back(pcl::PointXYZ(new_point.x() / weight_sum,
                                                new_point.y() / weight_sum,
                                                new_point.z() / weight_sum));
  }

  // With new vertices, construct new polygon mesh
  pcl::PolygonMesh new_mesh;
  new_mesh.polygons = original_mesh.polygons;
  pcl::toPCLPointCloud2(new_vertices, new_mesh.cloud);

  return new_mesh;
}

}  // namespace mesher_mapper