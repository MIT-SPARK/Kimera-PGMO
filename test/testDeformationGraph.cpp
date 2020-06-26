/**
 * @file   testDeformationGraph.cpp
 * @brief  Unit-tests for the deformation graph class
 * @author Yun Chang
 */

#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/PolygonMesh.h>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/DeformationGraph.h"
#include "test_config.h"

namespace mesher_mapper {

pcl::PolygonMesh createMeshTriangle() {
  // Create simple pcl mesh with one triangle
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZ> ptcld;
  ptcld.points.push_back(pcl::PointXYZ(0, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(1, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(0, 1, 0));
  pcl::toPCLPointCloud2(ptcld, mesh.cloud);

  pcl::Vertices tri_1;
  tri_1.vertices = std::vector<uint>{0, 1, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1};

  return mesh;
}

pcl::PolygonMesh SimpleMesh() {
  // Create simple pcl mesh
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZ> ptcld;
  ptcld.points.push_back(pcl::PointXYZ(0, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(1, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(0, 1, 0));
  ptcld.points.push_back(pcl::PointXYZ(1, 1, 0));
  ptcld.points.push_back(pcl::PointXYZ(0, 0, 1));
  pcl::toPCLPointCloud2(ptcld, mesh.cloud);

  pcl::Vertices tri_1, tri_2, tri_3, tri_4;
  tri_1.vertices = std::vector<uint>{0, 1, 2};
  tri_2.vertices = std::vector<uint>{1, 3, 2};
  tri_3.vertices = std::vector<uint>{0, 1, 4};
  tri_4.vertices = std::vector<uint>{0, 4, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1, tri_2, tri_3, tri_4};

  return mesh;
}

bool ComparePointcloud(const pcl::PointCloud<pcl::PointXYZ>& cloud1,
                       const pcl::PointCloud<pcl::PointXYZ>& cloud2,
                       double precision = 0.0) {
  if (cloud1.points.size() != cloud2.points.size()) return false;
  for (size_t i = 0; i < cloud1.points.size(); i++) {
    pcl::PointXYZ p1 = cloud1.points[i];
    pcl::PointXYZ p2 = cloud2.points[i];
    if (abs(p1.x - p2.x) > precision) return false;
    if (abs(p1.y - p2.y) > precision) return false;
    if (abs(p1.z - p2.z) > precision) return false;
  }
  return true;
}

TEST(DeformationGraph, reconstructMesh) {
  DeformationGraph graph;
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  // deform mesh
  graph.updateMesh(simple_mesh);
  graph.update();

  // First try deform with k = 1, should not change
  pcl::PolygonMesh new_mesh = graph.deformMesh(original_mesh, 1);
  pcl::PointCloud<pcl::PointXYZRGBA> deformed_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, deformed_vertices);
  EXPECT_EQ(5, deformed_vertices.points.size());
  EXPECT_EQ(1, deformed_vertices.points[1].x);
  EXPECT_EQ(1, deformed_vertices.points[2].y);
  EXPECT_EQ(1, deformed_vertices.points[4].z);
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);

  // Try with k = 2
  new_mesh = graph.deformMesh(original_mesh, 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, deformed_vertices);
  EXPECT_EQ(5, deformed_vertices.points.size());
  EXPECT_EQ(1, deformed_vertices.points[1].x);
  EXPECT_EQ(1, deformed_vertices.points[2].y);
  EXPECT_EQ(1, deformed_vertices.points[4].z);
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(DeformationGraph, deformMeshtranslation) {
  DeformationGraph graph;
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  // deform mesh
  graph.updateMesh(simple_mesh);
  geometry_msgs::Pose distortion;
  distortion.position.x = 1.5;
  graph.update();
  graph.addMeasurement(1, distortion);
  graph.optimize();

  pcl::PointCloud<pcl::PointXYZ> original_vertices, expected_vertices;
  pcl::fromPCLPointCloud2(original_mesh.cloud, original_vertices);
  for (pcl::PointXYZ p : original_vertices.points) {
    expected_vertices.push_back(pcl::PointXYZ(p.x + 0.5, p.y, p.z));
  }
  // First try deform with k = 1, should not change
  pcl::PolygonMesh new_mesh = graph.deformMesh(original_mesh, 1);
  pcl::PointCloud<pcl::PointXYZ> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);

  // Try with k = 2
  new_mesh = graph.deformMesh(original_mesh, 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(DeformationGraph, deformMesh) {
  pcl::PolygonMeshPtr cube_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/cube.ply", cube_mesh);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();
  // deform mesh
  DeformationGraph graph;
  graph.updateMesh(simple_mesh);
  geometry_msgs::Pose distortion;
  distortion.position.x = -0.5;
  graph.update();
  graph.addMeasurement(0, distortion);
  graph.optimize();
  pcl::PointCloud<pcl::PointXYZ> original_vertices, expected_vertices;
  pcl::fromPCLPointCloud2(cube_mesh->cloud, original_vertices);
  for (pcl::PointXYZ p : original_vertices.points) {
    expected_vertices.push_back(pcl::PointXYZ(p.x - 0.5, p.y, p.z));
  }
  // Try with k = 3
  pcl::PolygonMesh new_mesh = graph.deformMesh(*cube_mesh, 3);
  pcl::PointCloud<pcl::PointXYZ> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(cube_mesh->polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(cube_mesh->polygons[3].vertices, new_mesh.polygons[3].vertices);

  // deform mesh again
  geometry_msgs::Pose distortion2;
  distortion2.position.x = 1.5;
  graph.addMeasurement(1, distortion2);
  graph.optimize();
  // Try with k = 3
  new_mesh = graph.deformMesh(*cube_mesh, 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_NEAR(-0.5, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(1.273, actual_vertices.points[1].x, 0.001);

  EXPECT_EQ(cube_mesh->polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(cube_mesh->polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(DeformationGraph, updateMesh) {
  DeformationGraph graph;
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  graph.updateMesh(simple_mesh);
  graph.update();

  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getVertices().points[0].x);
  EXPECT_EQ(1, graph.getVertices().points[2].y);

  Vertices new_node_valences{0, 2};
  graph.addNode(pcl::PointXYZ(2, 2, 2), new_node_valences);
  graph.update();

  EXPECT_EQ(4, graph.getNumVertices());
  EXPECT_EQ(2, graph.getVertices().points[3].y);
  Graph base_graph = graph.getGraph();
  Vertex new_key = gtsam::Symbol('n', 0).key();

  std::vector<Edge> base_edges = base_graph.getEdges();
  EXPECT_EQ(10, base_edges.size());
  EXPECT_EQ(Edge(0, 1), base_edges[0]);
  EXPECT_EQ(Edge(0, new_key), base_edges[2]);
  EXPECT_EQ(Edge(1, 2), base_edges[4]);
  EXPECT_EQ(Edge(new_key, 2), base_edges[9]);

  Vertices new_node_valences_2{3};
  graph.addNode(pcl::PointXYZ(2, 3, 4), new_node_valences_2, true);
  graph.updateMesh(original_mesh);
  graph.update();

  EXPECT_EQ(7, graph.getNumVertices());
  EXPECT_EQ(2, graph.getVertices().points[5].y);
  EXPECT_EQ(3, graph.getVertices().points[6].y);
  EXPECT_EQ(4, graph.getVertices().points[6].z);
  base_graph = graph.getGraph();
  Vertex new_key_2 = gtsam::Symbol('n', 1).key();

  base_edges = base_graph.getEdges();
  EXPECT_EQ(24, base_edges.size());
  EXPECT_EQ(Edge(0, 1), base_edges[0]);
  EXPECT_EQ(Edge(0, new_key), base_edges[3]);
  EXPECT_EQ(Edge(1, 0), base_edges[4]);
  EXPECT_EQ(Edge(3, new_key_2), base_edges[15]);
  EXPECT_EQ(Edge(new_key, new_key_2), base_edges[21]);
}

TEST(DeformationGraph, addNodeMeasurement) {
  DeformationGraph graph;
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  pcl::PolygonMesh original_mesh = SimpleMesh();

  graph.updateMesh(simple_mesh);
  Vertices new_node_valences{0, 2};
  graph.addNode(pcl::PointXYZ(2, 2, 2), new_node_valences);
  graph.update();

  EXPECT_EQ(4, graph.getNumVertices());
  EXPECT_EQ(2, graph.getVertices().points[3].y);
  EXPECT_EQ(0, graph.getVertices().points[0].x);

  // Add node measurement
  graph.addNodeMeasurement(0,
                           gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2, 2)));
  graph.optimize();
  pcl::PolygonMesh new_mesh = graph.deformMesh(original_mesh, 1);
  pcl::PointCloud<pcl::PointXYZ> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_EQ(1.0, actual_vertices.points[0].x);
  EXPECT_EQ(1.0, actual_vertices.points[4].x);
  EXPECT_EQ(1.0, actual_vertices.points[4].z);

  // clear measurmenet
  graph.clearMeasurements();
  graph.addNodeMeasurement(
      0, gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2)));
  graph.optimize();
  new_mesh = graph.deformMesh(simple_mesh, 1);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_NEAR(4, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(4, actual_vertices.points[0].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[0].z, 0.001);
  EXPECT_NEAR(3, actual_vertices.points[1].x, 0.001);
  EXPECT_NEAR(3, actual_vertices.points[2].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[1].z, 0.001);
  EXPECT_NEAR(4, actual_vertices.points[2].x, 0.001);
}

}  // namespace mesher_mapper