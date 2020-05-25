/**
 * @file   DeformationSolver.h
 * @brief  Deforms mesh based on lates optimized trajectory
 * @author Yun Chang
 */
#include "mesher_mapper/DeformationGraph.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#define SLOW_BUT_CORRECT_BETWEENFACTOR

#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>

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

void DeformationGraph::loopClose(
    Vertex v1,
    Vertex v2,
    pcl::PointCloud<pcl::PointXYZ>::Ptr new_vertex_positions) {
  gtsam::NonlinearFactorGraph factors_to_optimize;
  factors_to_optimize.add(consistency_factors_);
  // noise for loop closure
  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);
  gtsam::BetweenFactor<gtsam::Pose3> loop_factor(v1, v2, gtsam::Pose3(), noise);
  factors_to_optimize.add(loop_factor);

  // optimize
  gtsam::LevenbergMarquardtParams params;
  params.diagonalDamping = true;
  gtsam::Values new_values =
      gtsam::LevenbergMarquardtOptimizer(factors_to_optimize, values_, params)
          .optimize();

  new_vertex_positions.reset(new pcl::PointCloud<pcl::PointXYZ>);
  for (Vertex v : graph_.getVertices()) {
    pcl::PointXYZ p = vertices_.points[v];
    gtsam::Point3 t = new_values.at<gtsam::Pose3>(v).translation();
    p.x = p.x + t.x();
    p.y = p.y + t.y();
    p.z = p.z + t.z();
    new_vertex_positions->points.push_back(p);
  }
}

}  // namespace mesher_mapper