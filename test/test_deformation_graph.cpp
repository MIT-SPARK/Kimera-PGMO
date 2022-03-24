/**
 * @file   test_deformation_graph.cpp
 * @brief  Unit-tests for the deformation graph class
 * @author Yun Chang
 */

#include "gtest/gtest.h"

#include <geometry_msgs/Pose.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/PolygonMesh.h>

#include <KimeraRPGO/SolverParams.h>

#include "kimera_pgmo/DeformationGraph.h"
#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/CommonStructs.h"
#include "test_config.h"

namespace kimera_pgmo {

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

  pcl::PointCloud<pcl::PointXYZRGBA> ptcld;
  pcl::PointXYZRGBA v0, v1, v2, v3, v4;
  v0.x = 0;
  v0.y = 0;
  v0.z = 0;
  v0.r = 23;
  v0.g = 24;
  v0.b = 122;
  v0.a = 255;

  v1.x = 1;
  v1.y = 0;
  v1.z = 0;
  v1.r = 33;
  v1.g = 34;
  v1.b = 52;
  v1.a = 255;

  v2.x = 0;
  v2.y = 1;
  v2.z = 0;
  v2.r = 12;
  v2.g = 144;
  v2.b = 22;
  v2.a = 255;

  v3.x = 1;
  v3.y = 1;
  v3.z = 0;
  v3.r = 0;
  v3.g = 14;
  v3.b = 0;
  v3.a = 255;

  v4.x = 0;
  v4.y = 0;
  v4.z = 1;
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

bool ComparePointcloud(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud1,
                       const pcl::PointCloud<pcl::PointXYZRGBA>& cloud2,
                       double precision = 0.0) {
  if (cloud1.points.size() != cloud2.points.size()) return false;
  for (size_t i = 0; i < cloud1.points.size(); i++) {
    pcl::PointXYZRGBA p1 = cloud1.points[i];
    pcl::PointXYZRGBA p2 = cloud2.points[i];
    if (abs(p1.x - p2.x) > precision) return false;
    if (abs(p1.y - p2.y) > precision) return false;
    if (abs(p1.z - p2.z) > precision) return false;
    if (p1.r != p2.r) return false;
    if (p1.g != p2.g) return false;
    if (p1.b != p2.b) return false;
    if (p1.a != p2.a) return false;
  }
  return true;
}

void MeshToEdgesAndNodes(
    const pcl::PolygonMesh& mesh,
    const char& prefix,
    gtsam::Values* mesh_nodes,
    std::vector<std::pair<gtsam::Key, gtsam::Key> >* mesh_edges) {
  pcl::PointCloud<pcl::PointXYZ> vertices;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices);
  std::vector<size_t> mesh_indices(vertices.size());
  std::iota(std::begin(mesh_indices), std::end(mesh_indices), 0);

  Graph graph_struct;
  std::vector<Edge> graph_mesh_edges =
      graph_struct.addPointsAndSurfaces(mesh_indices, mesh.polygons);

  for (auto i : mesh_indices) {
    mesh_nodes->insert(gtsam::Symbol(prefix, i),
                       gtsam::Pose3(gtsam::Rot3(), PclToGtsam(vertices.at(i))));
  }

  for (auto e : graph_mesh_edges) {
    mesh_edges->push_back(std::pair<gtsam::Key, gtsam::Key>(
        gtsam::Symbol(prefix, e.first), gtsam::Symbol(prefix, e.second)));
  }
}

void SetUpDeformationGraph(DeformationGraph* graph) {
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(100, 100, 100, 100,
                                  KimeraRPGO::Verbosity::UPDATE);
  graph->initialize(pgo_params);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();

  gtsam::Values mesh_nodes;
  std::vector<std::pair<gtsam::Key, gtsam::Key> > mesh_edges;
  MeshToEdgesAndNodes(simple_mesh, 'v', &mesh_nodes, &mesh_edges);

  std::vector<size_t> added_node_indices;
  graph->addNewMeshEdgesAndNodes(mesh_edges, mesh_nodes, ros::Time(0),
                                 &added_node_indices);
}

void SetUpOriginalMesh(pcl::PolygonMesh* mesh,
                       std::vector<ros::Time>* stamps,
                       std::vector<int>* indices) {
  *mesh = SimpleMesh();
  size_t num_vertices = mesh->cloud.width * mesh->cloud.height;
  for (size_t i = 0; i < num_vertices; i++) {
    stamps->push_back(ros::Time(0));
    if (i % 2 == 0) {
      indices->push_back(-1);
    } else {
      indices->push_back(0);
    }
  }
}

TEST(test_deformation_graph, addNewMeshEdgesAndNodes) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  // Check sizes
  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, graph.getInitialPositionVertex('v', 2).y());

  // Check that the factors are added
  gtsam::Values values = graph.getGtsamValues();
  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(6), factors.size());
  EXPECT_EQ(size_t(3), values.size());
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[0]));
  DeformationEdgeFactor factor =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[0]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), factor.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor.toPoint()));
}

TEST(test_deformation_graph, reconstructMesh) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh original_mesh;
  std::vector<ros::Time> original_mesh_stamps;
  std::vector<int> original_mesh_inds;
  SetUpOriginalMesh(&original_mesh, &original_mesh_stamps, &original_mesh_inds);

  // First try deform with k = 1, should not change
  pcl::PolygonMesh new_mesh = graph.deformMesh(
      original_mesh, original_mesh_stamps, original_mesh_inds, 'v', 1);
  pcl::PointCloud<pcl::PointXYZRGBA> deformed_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, deformed_vertices);
  EXPECT_EQ(5, deformed_vertices.points.size());
  EXPECT_EQ(1, deformed_vertices.points[1].x);
  EXPECT_EQ(1, deformed_vertices.points[2].y);
  EXPECT_EQ(1, deformed_vertices.points[4].z);
  EXPECT_EQ(33, deformed_vertices.points[1].r);
  EXPECT_EQ(144, deformed_vertices.points[2].g);
  EXPECT_EQ(255, deformed_vertices.points[3].a);
  EXPECT_EQ(144, deformed_vertices.points[4].r);
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);

  // Try with k = 2
  new_mesh = graph.deformMesh(
      original_mesh, original_mesh_stamps, original_mesh_inds, 'v', 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, deformed_vertices);
  EXPECT_EQ(5, deformed_vertices.points.size());
  EXPECT_EQ(1, deformed_vertices.points[1].x);
  EXPECT_EQ(1, deformed_vertices.points[2].y);
  EXPECT_EQ(1, deformed_vertices.points[4].z);
  EXPECT_EQ(33, deformed_vertices.points[1].r);
  EXPECT_EQ(144, deformed_vertices.points[2].g);
  EXPECT_EQ(255, deformed_vertices.points[3].a);
  EXPECT_EQ(144, deformed_vertices.points[4].r);
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(test_deformation_graph, deformMeshtranslation) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh original_mesh;
  std::vector<ros::Time> original_mesh_stamps;
  std::vector<int> original_mesh_inds;
  SetUpOriginalMesh(&original_mesh, &original_mesh_stamps, &original_mesh_inds);

  // deform mesh
  geometry_msgs::Pose distortion;
  distortion.position.x = 1.5;
  graph.addMeasurement(1, distortion, 'v');

  pcl::PointCloud<pcl::PointXYZRGBA> original_vertices, expected_vertices;
  pcl::fromPCLPointCloud2(original_mesh.cloud, original_vertices);
  for (pcl::PointXYZRGBA p : original_vertices.points) {
    pcl::PointXYZRGBA new_point;
    new_point.x = p.x + 0.5;
    new_point.y = p.y;
    new_point.z = p.z;
    new_point.r = p.r;
    new_point.g = p.g;
    new_point.b = p.b;
    new_point.a = p.a;
    expected_vertices.push_back(new_point);
  }
  // First try deform with k = 1, should not change
  pcl::PolygonMesh new_mesh = graph.deformMesh(
      original_mesh, original_mesh_stamps, original_mesh_inds, 'v', 1);
  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);

  // Try with k = 2
  new_mesh = graph.deformMesh(
      original_mesh, original_mesh_stamps, original_mesh_inds, 'v', 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(original_mesh.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original_mesh.polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(test_deformation_graph, deformMesh) {
  pcl::PolygonMeshPtr cube_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/cube.ply", cube_mesh);

  size_t num_vertices = cube_mesh->cloud.width * cube_mesh->cloud.height;
  std::vector<ros::Time> cube_mesh_stamps;
  std::vector<int> cube_mesh_inds;
  for (size_t i = 0; i < num_vertices; i++) {
    cube_mesh_stamps.push_back(ros::Time(0));
    cube_mesh_inds.push_back(-1);
  }

  // deform mesh
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  geometry_msgs::Pose distortion;
  distortion.position.x = -0.5;
  graph.addMeasurement(0, distortion, 'v');
  pcl::PointCloud<pcl::PointXYZRGBA> original_vertices, expected_vertices;
  pcl::fromPCLPointCloud2(cube_mesh->cloud, original_vertices);
  for (pcl::PointXYZRGBA p : original_vertices.points) {
    pcl::PointXYZRGBA new_point;
    new_point.x = p.x - 0.5;
    new_point.y = p.y;
    new_point.z = p.z;
    new_point.r = p.r;
    new_point.g = p.g;
    new_point.b = p.b;
    new_point.a = p.a;
    expected_vertices.push_back(new_point);
  }
  // Try with k = 3
  pcl::PolygonMesh new_mesh =
      graph.deformMesh(*cube_mesh, cube_mesh_stamps, cube_mesh_inds, 'v', 3);
  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(cube_mesh->polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(cube_mesh->polygons[3].vertices, new_mesh.polygons[3].vertices);

  // deform mesh again
  geometry_msgs::Pose distortion2;
  distortion2.position.x = 1.5;
  graph.addMeasurement(1, distortion2, 'v');
  // Try with k = 3
  new_mesh =
      graph.deformMesh(*cube_mesh, cube_mesh_stamps, cube_mesh_inds, 'v', 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_NEAR(-0.5, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(1.273, actual_vertices.points[1].x, 0.001);

  EXPECT_EQ(cube_mesh->polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(cube_mesh->polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(test_deformation_graph, updateMesh) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, graph.getInitialPositionVertex('v', 2).y());

  // Check that the factors are added
  gtsam::Values values = graph.getGtsamValues();
  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(6), factors.size());
  EXPECT_EQ(size_t(3), values.size());
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[0]));
  DeformationEdgeFactor factor =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[0]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), factor.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor.toPoint()));

  Vertices new_node_valences{0, 2};
  graph.addNewNode(gtsam::Symbol('a', 0),
                   gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                   false);
  graph.addNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');

  // Check that the factors are added correctly
  values = graph.getGtsamValues();
  factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(10), factors.size());
  EXPECT_EQ(size_t(4), values.size());
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[9]));
  DeformationEdgeFactor factor6 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[6]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), factor6.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(0, 0, 0), factor6.toPoint()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor6.front());
  EXPECT_EQ(gtsam::Symbol('v', 0).key(), factor6.back());

  Vertices new_node_valences_2{2};
  graph.addNewBetween(
      gtsam::Symbol('a', 0),
      gtsam::Symbol('a', 1),
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(0, 1, 2)),
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(2, 3, 4)));
  graph.addNodeValence(gtsam::Symbol('a', 1), new_node_valences_2, 'v');

  // Check that the factors are added
  values = graph.getGtsamValues();
  factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(13), factors.size());
  EXPECT_EQ(size_t(5), values.size());
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[12]));
  DeformationEdgeFactor factor12 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[12]);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)),
                          factor12.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 3, 4), factor12.toPoint()));
  EXPECT_EQ(gtsam::Symbol('v', 2).key(), factor12.front());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor12.back());
}

TEST(test_deformation_graph, addNodeMeasurements) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();
  size_t num_vertices = simple_mesh.cloud.width * simple_mesh.cloud.height;
  std::vector<ros::Time> simple_mesh_stamps;
  std::vector<int> simple_mesh_inds;
  for (size_t i = 0; i < num_vertices; i++) {
    simple_mesh_stamps.push_back(ros::Time(0));
    simple_mesh_inds.push_back(-1);
  }

  Vertices new_node_valences{0, 2};
  graph.addNewNode(
      gtsam::Symbol('a', 0),
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2)),
      false);
  graph.addNewNode(gtsam::Symbol('a', 1),
                   gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                   false);
  graph.addNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');

  // Check factors added
  gtsam::Values values = graph.getGtsamValues();
  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();

  // Add node measurement
  std::vector<std::pair<gtsam::Key, gtsam::Pose3> > measurements;
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      gtsam::Symbol('a', 0),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2))));
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      gtsam::Symbol('a', 1),
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2))));

  graph.addNodeMeasurements(measurements);

  factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(12), factors.size());
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[10]));
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[11]));

  pcl::PolygonMesh new_mesh = graph.deformMesh(
      simple_mesh, simple_mesh_stamps, simple_mesh_inds, 'v', 1);
  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_NEAR(4, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(4, actual_vertices.points[0].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[0].z, 0.001);
  EXPECT_NEAR(3, actual_vertices.points[1].x, 0.001);
  EXPECT_NEAR(3, actual_vertices.points[2].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[1].z, 0.001);
  EXPECT_NEAR(4, actual_vertices.points[2].x, 0.001);
}

TEST(test_deformation_graph, removePriorsWithPrefix) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();
  size_t num_vertices = simple_mesh.cloud.width * simple_mesh.cloud.height;
  std::vector<ros::Time> simple_mesh_stamps;
  std::vector<int> simple_mesh_inds;
  for (size_t i = 0; i < num_vertices; i++) {
    simple_mesh_stamps.push_back(ros::Time(0));
    simple_mesh_inds.push_back(-1);
  }

  Vertices new_node_valences{0, 2};
  graph.addNewNode(
      gtsam::Symbol('a', 0),
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2)),
      false);
  graph.addNewNode(gtsam::Symbol('a', 1),
                   gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                   false);
  graph.addNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');

  // Add node measurement
  std::vector<std::pair<gtsam::Key, gtsam::Pose3> > measurements;
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      gtsam::Symbol('a', 0),
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2))));
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      gtsam::Symbol('a', 1),
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2))));

  graph.addNodeMeasurements(measurements);

  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(12), factors.size());
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[10]));
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[11]));

  pcl::PolygonMesh new_mesh = graph.deformMesh(
      simple_mesh, simple_mesh_stamps, simple_mesh_inds, 'v', 1);
  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_NEAR(4, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(4, actual_vertices.points[0].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[0].z, 0.001);
  EXPECT_NEAR(3, actual_vertices.points[1].x, 0.001);
  EXPECT_NEAR(3, actual_vertices.points[2].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[1].z, 0.001);
  EXPECT_NEAR(4, actual_vertices.points[2].x, 0.001);

  // Up to here should be same as last test case
  // Remove priors
  graph.removePriorsWithPrefix('a');

  factors = graph.getGtsamFactors();
  EXPECT_EQ(size_t(10), factors.size());

  // Add another prior to see if mesh reset
  std::vector<std::pair<gtsam::Key, gtsam::Pose3> > priors;
  priors.push_back(
      {gtsam::Symbol('a', 0),
       gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2))});
  graph.addNodeMeasurements(priors);

  new_mesh = graph.deformMesh(
      simple_mesh, simple_mesh_stamps, simple_mesh_inds, 'v', 1);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_NEAR(0, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[0].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[0].z, 0.001);
  EXPECT_NEAR(1, actual_vertices.points[1].x, 0.001);
  EXPECT_NEAR(1, actual_vertices.points[2].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[1].z, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[2].x, 0.001);
}

TEST(test_deformation_graph, addNewBetween) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh original_mesh;
  std::vector<ros::Time> original_mesh_stamps;
  std::vector<int> original_mesh_inds;
  SetUpOriginalMesh(&original_mesh, &original_mesh_stamps, &original_mesh_inds);

  Vertices new_node_valences{0, 2};
  graph.addNewNode(gtsam::Symbol('a', 0),
                   gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                   true);
  graph.addNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');
  graph.addNewBetween(gtsam::Symbol('a', 0),
                      gtsam::Symbol('a', 1),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)));

  // Check added factors
  gtsam::Values values = graph.getGtsamValues();
  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(12), factors.size());
  EXPECT_EQ(size_t(5), values.size());

  // Check the between factor
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[0]));
  gtsam::BetweenFactor<gtsam::Pose3> factor0 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[0]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)), factor0.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor0.front());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor0.back());

  // Check the prior factor
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[7]));
  gtsam::PriorFactor<gtsam::Pose3> factor7 =
      *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
          factors[7]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), factor7.prior()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor7.key());

  // Check deformation edge factor
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[1]));
  DeformationEdgeFactor factor1 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[1]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), factor1.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor1.toPoint()));
  EXPECT_EQ(gtsam::Symbol('v', 0).key(), factor1.front());
  EXPECT_EQ(gtsam::Symbol('v', 1).key(), factor1.back());

  // Expect no change
  pcl::PolygonMesh new_mesh = graph.deformMesh(
      original_mesh, original_mesh_stamps, original_mesh_inds, 'v', 1);
  pcl::PointCloud<pcl::PointXYZ> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_EQ(0.0, actual_vertices.points[0].x);
  EXPECT_EQ(1.0, actual_vertices.points[3].y);
  EXPECT_EQ(1.0, actual_vertices.points[4].z);

  std::vector<gtsam::Pose3> traj = graph.getOptimizedTrajectory('a');
  EXPECT_EQ(2, traj.size());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), traj[0]));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)), traj[1]));

  graph.addNewBetween(gtsam::Symbol('a', 1),
                      gtsam::Symbol('a', 2),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)));
  graph.addNewBetween(gtsam::Symbol('a', 0),
                      gtsam::Symbol('a', 2),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));

  Vertices new_node_valences_2{2};
  graph.addNodeValence(gtsam::Symbol('a', 2), new_node_valences_2, 'v');

  // Check added factors
  values = graph.getGtsamValues();
  factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(16), factors.size());
  EXPECT_EQ(size_t(6), values.size());

  // Check the between factor
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[1]));
  gtsam::BetweenFactor<gtsam::Pose3> new1 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[1]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)),
      new1.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), new1.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), new1.back());

  // Check the between factor
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[15]));
  gtsam::BetweenFactor<gtsam::Pose3> new2 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[15]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), new2.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), new2.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), new2.back());

  // Check deformation edge factor
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[13]));
  DeformationEdgeFactor factor13 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[13]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)),
      factor13.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(0, 1, 0), factor13.toPoint()));
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), factor13.front());
  EXPECT_EQ(gtsam::Symbol('v', 2).key(), factor13.back());

  traj = graph.getOptimizedTrajectory('a');
  EXPECT_EQ(3, traj.size());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.05, 2.05)),
      traj[2],
      0.05));
}

TEST(test_deformation_graph, addTemporary) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  Vertices new_node_valences{0, 2};
  graph.addNewNode(gtsam::Symbol('a', 0),
                   gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                   true);
  graph.addNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');
  graph.addNewBetween(gtsam::Symbol('a', 0),
                      gtsam::Symbol('a', 1),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)));

  graph.addNewBetween(gtsam::Symbol('a', 1),
                      gtsam::Symbol('a', 2),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)));

  Vertices new_node_valences_2{2};
  graph.addNodeValence(gtsam::Symbol('a', 2), new_node_valences_2, 'v');

  // Check added factors
  gtsam::Values values = graph.getGtsamValues();
  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(15), factors.size());
  EXPECT_EQ(size_t(6), values.size());

  std::vector<gtsam::Pose3> traj = graph.getOptimizedTrajectory('a');
  EXPECT_EQ(3, traj.size());

  // Add temporary nodes and edges
  graph.addNewTempNode(gtsam::Symbol('p', 0), gtsam::Pose3(), false);
  graph.addNewTempNode(gtsam::Symbol('p', 1),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)),
                       false);
  graph.addNewTempBetween(gtsam::Symbol('p', 0),
                          gtsam::Symbol('p', 1),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)));
  Vertices temp_node_valences{1, 2};
  graph.addTempNodeValence(gtsam::Symbol('p', 0), temp_node_valences, 'v');

  graph.optimize();

  // Check added factors
  values = graph.getGtsamValues();
  factors = graph.getGtsamFactors();
  gtsam::Values temp_values = graph.getGtsamTempValues();
  gtsam::NonlinearFactorGraph temp_factors = graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(15), factors.size());
  EXPECT_EQ(size_t(6), values.size());

  EXPECT_EQ(size_t(5), temp_factors.size());
  EXPECT_EQ(size_t(2), temp_values.size());

  traj = graph.getOptimizedTrajectory('a');
  EXPECT_EQ(3, traj.size());

  graph.clearTemporaryStructures();
  graph.optimize();
  temp_values = graph.getGtsamTempValues();
  temp_factors = graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(0), temp_factors.size());
  EXPECT_EQ(size_t(0), temp_values.size());

  // Re-ad temporary nodes and edges
  graph.addNewTempNode(gtsam::Symbol('p', 0), gtsam::Pose3(), false);
  Vertices temp_node_valences_2{0, 1, 2};
  graph.addTempNodeValence(gtsam::Symbol('p', 0), temp_node_valences_2, 'v');
  graph.optimize();

  // Check added factors
  temp_values = graph.getGtsamTempValues();
  temp_factors = graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(6), temp_factors.size());
  EXPECT_EQ(size_t(1), temp_values.size());
}

TEST(test_deformation_graph, addNewTempNodesValences) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  std::vector<gtsam::Key> temp_keys;
  std::vector<gtsam::Pose3> temp_key_poses;
  std::vector<Vertices> temp_valences;
  for (size_t i = 0; i < 3; i++) {
    temp_keys.push_back(gtsam::Symbol('p', i));
    temp_key_poses.push_back(gtsam::Pose3(
        gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0)));
    Vertices temp_node_valences{i};
    temp_valences.push_back(temp_node_valences);
  }

  graph.addNewTempNodesValences(
      temp_keys, temp_key_poses, temp_valences, 'v', false);

  // Check added factors
  gtsam::Values values = graph.getGtsamValues();
  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();

  EXPECT_EQ(size_t(6), factors.size());
  EXPECT_EQ(size_t(3), values.size());

  graph.optimize();

  // Check added factors
  values = graph.getGtsamValues();
  factors = graph.getGtsamFactors();
  gtsam::Values temp_values = graph.getGtsamTempValues();
  gtsam::NonlinearFactorGraph temp_factors = graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(6), factors.size());
  EXPECT_EQ(size_t(3), values.size());

  for (size_t i = 0; i < 3; i++) {
    gtsam::Pose3 node_pose(gtsam::Rot3(),
                           gtsam::Point3(static_cast<double>(i), 0, 0));
    gtsam::Pose3 vertex_pose = values.at<gtsam::Pose3>(gtsam::Symbol('v', i));
    EXPECT_TRUE(gtsam::assert_equal(
        node_pose, temp_values.at<gtsam::Pose3>(gtsam::Symbol('p', i))));
    DeformationEdgeFactor factor1 =
        *boost::dynamic_pointer_cast<DeformationEdgeFactor>(
            temp_factors[2 * i]);
    DeformationEdgeFactor factor2 =
        *boost::dynamic_pointer_cast<DeformationEdgeFactor>(
            temp_factors[2 * i + 1]);
    EXPECT_TRUE(gtsam::assert_equal(node_pose, factor1.fromPose()));
    EXPECT_TRUE(
        gtsam::assert_equal(node_pose.translation(), factor2.toPoint()));
    EXPECT_TRUE(
        gtsam::assert_equal(vertex_pose.translation(), factor1.toPoint()));
    EXPECT_TRUE(gtsam::assert_equal(vertex_pose, factor2.fromPose()));
    EXPECT_EQ(gtsam::Symbol('p', i).key(), factor1.front());
    EXPECT_EQ(gtsam::Symbol('v', i).key(), factor1.back());
    EXPECT_EQ(gtsam::Symbol('v', i).key(), factor2.front());
    EXPECT_EQ(gtsam::Symbol('p', i).key(), factor2.back());
  }

  graph.clearTemporaryStructures();
  graph.optimize();
  temp_values = graph.getGtsamTempValues();
  temp_factors = graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(0), temp_factors.size());
  EXPECT_EQ(size_t(0), temp_values.size());
}

TEST(test_deformation_graph, addNewTempEdges) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  std::vector<gtsam::Key> temp_keys;
  std::vector<gtsam::Pose3> temp_key_poses;
  std::vector<Vertices> temp_valences;
  for (size_t i = 0; i < 4; i++) {
    temp_keys.push_back(gtsam::Symbol('p', i));
    temp_key_poses.push_back(gtsam::Pose3(
        gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0)));
    Vertices temp_node_valences;
    temp_valences.push_back(temp_node_valences);
  }
  graph.addNewTempNodesValences(
      temp_keys, temp_key_poses, temp_valences, 'v', false);


  pose_graph_tools::PoseGraph temp_edges;
  for (size_t i = 0; i < 3; i++) {
    pose_graph_tools::PoseGraphEdge edge;
    edge.key_from = gtsam::Symbol('p', i);
    edge.key_to = gtsam::Symbol('p', i + 1);
    temp_edges.edges.push_back(edge);
  }

  graph.addNewTempEdges(temp_edges);

  // Check added factors
  gtsam::Values values = graph.getGtsamValues();
  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();
  gtsam::Values temp_values = graph.getGtsamTempValues();
  gtsam::NonlinearFactorGraph temp_factors = graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(3), values.size());
  EXPECT_EQ(size_t(6), factors.size());
  EXPECT_EQ(size_t(4), temp_values.size());
  EXPECT_EQ(size_t(3), temp_factors.size());

  for (size_t i = 0; i < 3; i++) {
    gtsam::BetweenFactor<gtsam::Pose3> factor =
        *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
            temp_factors[i]);
    EXPECT_EQ(gtsam::Symbol('p', i).key(), factor.front());
    EXPECT_EQ(gtsam::Symbol('p', i + 1).key(), factor.back());
  }

  graph.clearTemporaryStructures();
  graph.optimize();
  temp_values = graph.getGtsamTempValues();
  temp_factors = graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(0), temp_factors.size());
  EXPECT_EQ(size_t(0), temp_values.size());
}

TEST(test_deformation_graph, saveAndLoad) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  Vertices new_node_valences{0, 2};
  graph.addNewNode(gtsam::Symbol('a', 0),
                   gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                   true);
  graph.addNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');
  graph.addNewBetween(gtsam::Symbol('a', 0),
                      gtsam::Symbol('a', 1),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)));

  graph.addNewBetween(gtsam::Symbol('a', 1),
                      gtsam::Symbol('a', 2),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)),
                      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)));

  Vertices new_node_valences_2{2};
  graph.addNodeValence(gtsam::Symbol('a', 2), new_node_valences_2, 'v');

  // Add temporary nodes and edges
  graph.addNewTempNode(gtsam::Symbol('p', 0), gtsam::Pose3(), false);
  graph.addNewTempNode(gtsam::Symbol('p', 1),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)),
                       false);
  graph.addNewTempBetween(gtsam::Symbol('p', 0),
                          gtsam::Symbol('p', 1),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)));
  Vertices temp_node_valences{1, 2};
  graph.addTempNodeValence(gtsam::Symbol('p', 0), temp_node_valences, 'v');

  graph.optimize();

  // Check added factors
  gtsam::Values values = graph.getGtsamValues();
  gtsam::NonlinearFactorGraph factors = graph.getGtsamFactors();
  gtsam::Values temp_values = graph.getGtsamTempValues();
  gtsam::NonlinearFactorGraph temp_factors = graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(15), factors.size());
  EXPECT_EQ(size_t(6), values.size());

  EXPECT_EQ(size_t(5), temp_factors.size());
  EXPECT_EQ(size_t(2), temp_values.size());
  graph.save(std::string(DATASET_PATH) + "/graph.dgrf");
  DeformationGraph new_graph;
  new_graph.initialize(graph.getParams());
  new_graph.load(std::string(DATASET_PATH) + "/graph.dgrf");

  values = new_graph.getGtsamValues();
  factors = new_graph.getGtsamFactors();
  temp_values = new_graph.getGtsamTempValues();
  temp_factors = new_graph.getGtsamTempFactors();

  EXPECT_EQ(size_t(15), factors.size());
  EXPECT_EQ(size_t(6), values.size());

  EXPECT_EQ(size_t(5), temp_factors.size());
  EXPECT_EQ(size_t(2), temp_values.size());
  EXPECT_EQ(3, new_graph.getNumVertices());
  EXPECT_EQ(0, new_graph.getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, new_graph.getInitialPositionVertex('v', 2).y());
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
