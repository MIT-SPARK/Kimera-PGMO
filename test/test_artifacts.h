/**
 * @file   test_artifacts.h
 * @brief  Some artifacts used in unittests
 * @author Yun Chang
 */
#pragma once

#include <string>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pose_graph_tools/PoseGraph.h>

#include "kimera_pgmo/KimeraPgmoMesh.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/CommonStructs.h"

namespace kimera_pgmo {
pose_graph_tools::PoseGraph SingleOdomGraph(const ros::Time& stamp,
                                            const size_t& robot_id) {
  pose_graph_tools::PoseGraph inc_graph;

  pose_graph_tools::PoseGraphEdge e0;
  pose_graph_tools::PoseGraphNode n0, n1;
  e0.header.stamp = stamp;
  e0.key_from = 0;
  e0.key_to = 1;
  e0.robot_from = robot_id;
  e0.robot_to = robot_id;
  e0.pose.position.x = 1;
  e0.pose.orientation.w = 1;
  e0.type = pose_graph_tools::PoseGraphEdge::ODOM;
  e0.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                   0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                   0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

  n0.header.stamp = stamp;
  n0.key = 0;
  n0.robot_id = robot_id;
  n0.pose.orientation.w = 1;

  n1.header.stamp = stamp;
  n1.key = 1;
  n1.robot_id = robot_id;
  n1.pose.position.x = 1;
  n1.pose.orientation.w = 1;

  inc_graph.edges.push_back(e0);
  inc_graph.nodes.push_back(n0);
  inc_graph.nodes.push_back(n1);

  return inc_graph;
}

pose_graph_tools::PoseGraph OdomLoopclosureGraph(const ros::Time& stamp,
                                                 const size_t& robot_id) {
  pose_graph_tools::PoseGraph inc_graph;

  pose_graph_tools::PoseGraphEdge e1, e2;
  pose_graph_tools::PoseGraphNode n1, n2;

  e1.header.stamp = stamp;
  e1.key_from = 1;
  e1.key_to = 2;
  e1.robot_from = robot_id;
  e1.robot_to = robot_id;
  e1.pose.position.y = 1;
  e1.pose.orientation.w = 1;
  e1.type = pose_graph_tools::PoseGraphEdge::ODOM;
  e1.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                   0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                   0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

  e2.header.stamp = stamp;
  e2.key_from = 0;
  e2.key_to = 2;
  e2.robot_from = robot_id;
  e2.robot_to = robot_id;
  e2.pose.position.x = 1;
  e2.pose.position.y = 1;
  e2.pose.orientation.w = 1;
  e2.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
  e2.covariance = {0.1, 0, 0,   0, 0,   0, 0, 0.1, 0, 0,   0, 0,
                   0,   0, 0.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                   0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

  n1.header.stamp = stamp;
  n1.key = 1;
  n1.robot_id = robot_id;
  n1.pose.orientation.w = 1;

  n2.header.stamp = stamp;
  n2.key = 2;
  n2.robot_id = robot_id;
  n2.pose.position.x = 1;
  n2.pose.position.y = 1;
  n2.pose.orientation.w = 1;

  inc_graph.edges.push_back(e1);
  inc_graph.edges.push_back(e2);
  inc_graph.nodes.push_back(n1);
  inc_graph.nodes.push_back(n2);

  return inc_graph;
}

pose_graph_tools::PoseGraph OdomLoopclosureGraph2(const ros::Time& stamp,
                                                  const size_t& robot_id) {
  pose_graph_tools::PoseGraph inc_graph;

  pose_graph_tools::PoseGraphEdge e1, e2;
  pose_graph_tools::PoseGraphNode n1, n2;

  e1.header.stamp = stamp;
  e1.key_from = 2;
  e1.key_to = 3;
  e1.robot_from = robot_id;
  e1.robot_to = robot_id;
  e1.pose.position.z = 1;
  e1.pose.orientation.w = 1;
  e1.type = pose_graph_tools::PoseGraphEdge::ODOM;
  e1.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                   0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                   0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

  e2.header.stamp = stamp;
  e2.key_from = 0;
  e2.key_to = 3;
  e2.robot_from = robot_id;
  e2.robot_to = robot_id;
  e2.pose.position.z = -1;
  e2.pose.position.x = -1;
  e2.pose.orientation.w = 1;
  e2.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
  e2.covariance = {0.1, 0, 0,   0, 0,   0, 0, 0.1, 0, 0,   0, 0,
                   0,   0, 0.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                   0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

  n1.header.stamp = stamp;
  n1.key = 0;
  n1.robot_id = robot_id;
  n1.pose.orientation.w = 1;

  n2.header.stamp = stamp;
  n2.key = 3;
  n2.robot_id = robot_id;
  n2.pose.orientation.w = 1;

  inc_graph.edges.push_back(e1);
  inc_graph.edges.push_back(e2);
  inc_graph.nodes.push_back(n1);
  inc_graph.nodes.push_back(n2);

  return inc_graph;
}

pose_graph_tools::PoseGraph InterRobotLoopClosureGraph(const ros::Time& stamp,
                                                       const size_t& robot_1,
                                                       const size_t& robot_2) {
  pose_graph_tools::PoseGraph inc_graph;

  pose_graph_tools::PoseGraphEdge e1;

  e1.header.stamp = stamp;
  e1.key_from = 0;
  e1.key_to = 0;
  e1.robot_from = robot_1;
  e1.robot_to = robot_2;
  e1.pose.position.z = 1;
  e1.pose.orientation.w = 1;
  e1.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
  e1.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                   0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                   0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

  inc_graph.edges.push_back(e1);

  return inc_graph;
}

pcl::PolygonMesh createMesh(double t_x, double t_y, double t_z) {
  // Create simple pcl mesh
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZRGBA> ptcld;
  pcl::PointXYZRGBA v0, v1, v2, v3, v4;
  v0.x = 0 + t_x;
  v0.y = 0 + t_y;
  v0.z = 0 + t_z;
  v0.r = 23;
  v0.g = 24;
  v0.b = 122;
  v0.a = 255;

  v1.x = 1 + t_x;
  v1.y = 0 + t_y;
  v1.z = 0 + t_z;
  v1.r = 33;
  v1.g = 34;
  v1.b = 52;
  v1.a = 255;

  v2.x = 0 + t_x;
  v2.y = 1 + t_y;
  v2.z = 0 + t_z;
  v2.r = 12;
  v2.g = 144;
  v2.b = 22;
  v2.a = 255;

  v3.x = 1 + t_x;
  v3.y = 1 + t_y;
  v3.z = 0 + t_z;
  v3.r = 0;
  v3.g = 14;
  v3.b = 0;
  v3.a = 255;

  v4.x = 0 + t_x;
  v4.y = 0 + t_y;
  v4.z = 1 + t_z;
  v4.r = 144;
  v4.g = 0;
  v4.b = 12;
  v4.a = 255;

  ptcld.points.push_back(v0);
  ptcld.points.push_back(v1);
  ptcld.points.push_back(v2);
  ptcld.points.push_back(v3);
  ptcld.points.push_back(v4);
  pcl::toPCLPointCloud2(ptcld, mesh.cloud);

  pcl::Vertices tri_1, tri_2, tri_3, tri_4;
  tri_1.vertices = std::vector<uint>{0, 1, 2};
  tri_2.vertices = std::vector<uint>{1, 3, 2};
  tri_3.vertices = std::vector<uint>{0, 1, 4};
  tri_4.vertices = std::vector<uint>{0, 4, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1, tri_2, tri_3, tri_4};

  return mesh;
}

pose_graph_tools::PoseGraph processMeshToGraph(
    const pcl::PolygonMesh& mesh,
    const size_t& robot_id,
    const ros::Time& msg_time,
    const OctreeCompressionPtr compressor,
    Graph* graph) {
  // Add to deformation graph mesh compressor
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mesh_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr graph_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::fromPCLPointCloud2(mesh.cloud, *mesh_vertices);
  std::shared_ptr<std::vector<pcl::Vertices> > graph_triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<size_t> > graph_indices(
      new std::vector<size_t>);
  std::shared_ptr<std::unordered_map<size_t, size_t> > index_remappings =
      std::make_shared<std::unordered_map<size_t, size_t> >();

  compressor->compressAndIntegrate(*mesh_vertices,
                                   mesh.polygons,
                                   graph_vertices,
                                   graph_triangles,
                                   graph_indices,
                                   index_remappings,
                                   msg_time.toSec());

  const std::vector<Edge>& new_edges =
      graph->addPointsAndSurfaces(*graph_indices, *graph_triangles);
  compressor->getVertices(graph_vertices);

  // Create message
  pose_graph_tools::PoseGraph pose_graph_msg;
  pose_graph_msg.header.stamp = msg_time;

  // Encode the edges as factors
  for (auto e : new_edges) {
    pose_graph_tools::PoseGraphEdge pg_edge;
    pg_edge.header.stamp = msg_time;

    const size_t& from_node = e.first;
    const size_t& to_node = e.second;

    pg_edge.robot_from = robot_id;
    pg_edge.robot_to = robot_id;
    pg_edge.key_from = from_node;
    pg_edge.key_to = to_node;

    gtsam::Point3 from_node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices->at(from_node));
    gtsam::Point3 to_node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices->at(to_node));
    pg_edge.pose =
        GtsamToRos(gtsam::Pose3(gtsam::Rot3(), to_node_pos - from_node_pos));

    pg_edge.type = pose_graph_tools::PoseGraphEdge::MESH;

    // Add edge to pose graph
    pose_graph_msg.edges.push_back(pg_edge);
  }

  // Encode the new vertices as nodes
  for (auto n : *graph_indices) {
    pose_graph_tools::PoseGraphNode pg_node;
    pg_node.header.stamp = msg_time;
    pg_node.robot_id = robot_id;

    pg_node.key = n;

    gtsam::Point3 node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices->at(n));
    pg_node.pose = GtsamToRos(gtsam::Pose3(gtsam::Rot3(), node_pos));

    // Add node to pose graph
    pose_graph_msg.nodes.push_back(pg_node);
  }

  return pose_graph_msg;
}
}  // namespace kimera_pgmo