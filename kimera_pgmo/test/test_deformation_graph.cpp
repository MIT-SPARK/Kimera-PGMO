/**
 * @file   test_deformation_graph.cpp
 * @brief  Unit-tests for the deformation graph class
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <kimera_rpgo/rpgo.h>
#include <pcl/conversions.h>

#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/utils/common_functions.h"
#include "kimera_pgmo/utils/common_structs.h"
#include "kimera_pgmo/utils/mesh_io.h"
#include "pgmo_fixtures.h"
#include "test_config.h"

namespace kimera_pgmo {

namespace {
template <typename T, typename Ptr>
const T* cast_factor(const Ptr& ptr, bool should_throw = false) {
  if (!ptr) {
    if (should_throw) {
      throw std::runtime_error("bad pointer");
    }
    return nullptr;
  }

  auto cast = dynamic_cast<const T*>(ptr.get());
  if (!cast && should_throw) {
    throw std::runtime_error("bad cast");
  }

  return cast;
}
}  // namespace

pcl::PolygonMesh createMeshTriangle() {
  // Create simple pcl mesh with one triangle
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZ> ptcld;
  ptcld.points.push_back(pcl::PointXYZ(0, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(1, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(0, 1, 0));
  pcl::toPCLPointCloud2(ptcld, mesh.cloud);

  pcl::Vertices tri_1;
  tri_1.vertices = {0, 1, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1};

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

void MeshToEdgesAndNodes(const pcl::PolygonMesh& mesh,
                         const char& prefix,
                         gtsam::Values* mesh_nodes,
                         std::vector<std::pair<gtsam::Key, gtsam::Key>>* mesh_edges,
                         std::unordered_map<gtsam::Key, Timestamp>* node_stamps) {
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
    node_stamps->insert({gtsam::Symbol(prefix, i), 0});
  }

  for (auto e : graph_mesh_edges) {
    mesh_edges->push_back(std::pair<gtsam::Key, gtsam::Key>(
        gtsam::Symbol(prefix, e.first), gtsam::Symbol(prefix, e.second)));
  }
}

void SetUpDeformationGraph(DeformationGraph* graph,
                           bool add_mesh = true,
                           bool quiet = true) {
  if (!add_mesh) {
    return;
  }

  pcl::PolygonMesh simple_mesh = createMeshTriangle();
  gtsam::Values mesh_nodes;
  std::vector<std::pair<gtsam::Key, gtsam::Key>> mesh_edges;
  std::unordered_map<gtsam::Key, Timestamp> mesh_node_stamps;
  MeshToEdgesAndNodes(simple_mesh, 'v', &mesh_nodes, &mesh_edges, &mesh_node_stamps);

  std::vector<size_t> added_node_indices;
  std::vector<Timestamp> added_node_stamps;
  graph->processNewMeshEdgesAndNodes(mesh_edges,
                                     mesh_nodes,
                                     mesh_node_stamps,
                                     &added_node_indices,
                                     &added_node_stamps);
}

void SetUpOriginalMesh(pcl::PolygonMesh& mesh,
                       std::vector<Timestamp>* stamps,
                       std::vector<int>* indices) {
  mesh = test::createSimpleMesh();
  size_t num_vertices = mesh.cloud.width * mesh.cloud.height;
  for (size_t i = 0; i < num_vertices; i++) {
    stamps->push_back(0);
    if (i % 2 == 0) {
      indices->push_back(-1);
    } else {
      indices->push_back(0);
    }
  }
}

void OptimizeOnce(DeformationGraph& deformation_graph) {
  kimera_rpgo::RpgoConfig pgo_config;
  pgo_config.solver_config.setLeastSquaresParamsDefault();
  kimera_rpgo::Rpgo pgo(pgo_config);

  auto factors = *deformation_graph.getFactors();
  auto temp_factors = *deformation_graph.getTempFactors();
  auto values = *deformation_graph.getValues();
  auto temp_values = *deformation_graph.getTempValues();
  pgo.addFactors(factors);
  pgo.addValues(values);
  pgo.addFactors(temp_factors);
  pgo.addValues(temp_values);
  pgo.run();

  // Get estimates
  auto results = gtsam::Values(values);
  auto temp_results = gtsam::Values(temp_values);
  auto rpgo_result = pgo.getResult();
  for (const auto& key_val : rpgo_result) {
    if (results.exists(key_val.key)) {
      results.update(key_val.key, key_val.value);
    }
    if (temp_results.exists(key_val.key)) {
      temp_results.update(key_val.key, key_val.value);
    }
  }

  deformation_graph.updateValues(results);
  deformation_graph.updateTempValues(temp_results);

  // For testing purposes always recalculate vertices
  deformation_graph.setRecalculateVertices();
}

TEST(TestDeformationGraph, processNewMeshEdgesAndNodes) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  // Check sizes
  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, graph.getInitialPositionVertex('v', 2).y());

  // Check that the factors are added
  auto values = graph.getValues();
  auto factors = graph.getFactors();

  EXPECT_EQ(6u, factors->size());
  EXPECT_EQ(3u, values->size());
}

TEST(TestDeformationGraph, reconstructMesh) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh original_mesh;
  std::vector<Timestamp> original_mesh_stamps;
  std::vector<int> original_mesh_inds;
  SetUpOriginalMesh(original_mesh, &original_mesh_stamps, &original_mesh_inds);

  // First try deform with k = 1, should not change
  pcl::PolygonMesh new_mesh =
      graph.deformMesh(original_mesh, original_mesh_stamps, original_mesh_inds, 'v', 1);
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
  new_mesh =
      graph.deformMesh(original_mesh, original_mesh_stamps, original_mesh_inds, 'v', 2);
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

TEST(TestDeformationGraph, deformMeshTranslation) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  pcl::PolygonMesh original;
  std::vector<Timestamp> original_stamps;
  std::vector<int> original_inds;
  SetUpOriginalMesh(original, &original_stamps, &original_inds);

  // deform mesh
  gtsam::Pose3 distortion(gtsam::Rot3(), gtsam::Point3(1.5, 0.0, 0.0));
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> node_distortions;
  node_distortions.push_back({gtsam::Symbol('v', 1), distortion});
  graph.processNodeMeasurements(node_distortions, 1e-10);

  pcl::PointCloud<pcl::PointXYZRGBA> original_vertices, expected_vertices;
  pcl::fromPCLPointCloud2(original.cloud, original_vertices);
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
  OptimizeOnce(graph);
  auto new_mesh = graph.deformMesh(original, original_stamps, original_inds, 'v', 1);

  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(original.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original.polygons[3].vertices, new_mesh.polygons[3].vertices);

  // Try with k = 2
  new_mesh = graph.deformMesh(original, original_stamps, original_inds, 'v', 2);

  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(original.polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(original.polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(TestDeformationGraph, deformMesh) {
  pcl::PolygonMeshPtr cube_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/cube.ply", cube_mesh);

  size_t num_vertices = cube_mesh->cloud.width * cube_mesh->cloud.height;
  std::vector<Timestamp> cube_mesh_stamps;
  std::vector<int> cube_mesh_inds;
  for (size_t i = 0; i < num_vertices; i++) {
    cube_mesh_stamps.push_back(0);
    cube_mesh_inds.push_back(-1);
  }

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

  // deform mesh
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  gtsam::Pose3 distortion(gtsam::Rot3(), gtsam::Point3(-0.5, 0.0, 0.0));
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> node_distortions;
  node_distortions.push_back({gtsam::Symbol('v', 0), distortion});
  graph.processNodeMeasurements(node_distortions, 1e-10);

  // optimize
  OptimizeOnce(graph);

  // Try with k = 3
  auto new_mesh =
      graph.deformMesh(*cube_mesh, cube_mesh_stamps, cube_mesh_inds, 'v', 3);
  pcl::PointCloud<pcl::PointXYZRGBA> actual_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_TRUE(ComparePointcloud(expected_vertices, actual_vertices, 1e-6));
  EXPECT_EQ(cube_mesh->polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(cube_mesh->polygons[3].vertices, new_mesh.polygons[3].vertices);

  // deform mesh again
  gtsam::Pose3 distortion2(gtsam::Rot3(), gtsam::Point3(1.5, 0.0, 0.0));
  node_distortions.clear();
  node_distortions.push_back({gtsam::Symbol('v', 1), distortion2});
  graph.processNodeMeasurements(node_distortions, 1e-10);

  // optimize
  OptimizeOnce(graph);

  // Try with k = 3
  new_mesh = graph.deformMesh(*cube_mesh, cube_mesh_stamps, cube_mesh_inds, 'v', 2);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);
  EXPECT_NEAR(-0.5, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(1.273, actual_vertices.points[1].x, 0.001);
  EXPECT_EQ(cube_mesh->polygons[0].vertices, new_mesh.polygons[0].vertices);
  EXPECT_EQ(cube_mesh->polygons[3].vertices, new_mesh.polygons[3].vertices);
}

TEST(TestDeformationGraph, updateMesh) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, graph.getInitialPositionVertex('v', 2).y());

  // Check that the factors are added
  auto values = graph.getValues();
  auto factors = graph.getFactors();

  ASSERT_EQ(6u, factors->size());
  ASSERT_EQ(3u, values->size());
  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors->at(0)));
  DeformationEdgeFactor factor = *cast_factor<DeformationEdgeFactor>(factors->at(0));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor.measurement()));

  Vertices new_node_valences{0, 2};
  graph.processNewNode(gtsam::Symbol('a', 0),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                       false);
  graph.processNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');

  // Check that the factors are added correctly
  values = graph.getValues();
  factors = graph.getFactors();

  ASSERT_EQ(10u, factors->size());
  ASSERT_EQ(4u, values->size());
  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors->at(9)));
  DeformationEdgeFactor factor6 = *cast_factor<DeformationEdgeFactor>(factors->at(6));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(-2, -2, -2), factor6.measurement()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor6.front());
  EXPECT_EQ(gtsam::Symbol('v', 0).key(), factor6.back());

  Vertices new_node_valences_2{2};
  graph.processNewNode(gtsam::Symbol('a', 1),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)),
                       false);
  graph.processNewBetween(
      gtsam::Symbol('a', 0),
      gtsam::Symbol('a', 1),
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(0, 1, 2)));
  graph.processNodeValence(gtsam::Symbol('a', 1), new_node_valences_2, 'v');

  // Check that the factors are added
  values = graph.getValues();
  factors = graph.getFactors();

  ASSERT_EQ(13u, factors->size());
  ASSERT_EQ(5u, values->size());
  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors->at(12)));
  DeformationEdgeFactor factor12 = *cast_factor<DeformationEdgeFactor>(factors->at(12));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 2, 4), factor12.measurement()));
  EXPECT_EQ(gtsam::Symbol('v', 2).key(), factor12.front());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor12.back());
}

TEST(TestDeformationGraph, addNodeMeasurements) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();
  size_t num_vertices = simple_mesh.cloud.width * simple_mesh.cloud.height;
  std::vector<Timestamp> simple_mesh_stamps;
  std::vector<int> simple_mesh_inds;
  for (size_t i = 0; i < num_vertices; i++) {
    simple_mesh_stamps.push_back(0);
    simple_mesh_inds.push_back(-1);
  }

  Vertices new_node_valences{0, 2};
  graph.processNewNode(gtsam::Symbol('a', 0),
                       gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2)),
                       false);
  graph.processNewNode(gtsam::Symbol('a', 1),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                       false);
  graph.processNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');

  // Check factors added
  auto factors = graph.getFactors();

  // Add node measurement
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> measurements;
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2))));
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      gtsam::Symbol('a', 1),
      gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2))));

  graph.processNodeMeasurements(measurements);

  factors = graph.getFactors();

  EXPECT_EQ(12u, factors->size());
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(10)));
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(11)));

  OptimizeOnce(graph);

  pcl::PolygonMesh new_mesh =
      graph.deformMesh(simple_mesh, simple_mesh_stamps, simple_mesh_inds, 'v', 1);
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

TEST(TestDeformationGraph, removePriorsWithPrefix) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh simple_mesh = createMeshTriangle();
  size_t num_vertices = simple_mesh.cloud.width * simple_mesh.cloud.height;
  std::vector<Timestamp> simple_mesh_stamps;
  std::vector<int> simple_mesh_inds;
  for (size_t i = 0; i < num_vertices; i++) {
    simple_mesh_stamps.push_back(0);
    simple_mesh_inds.push_back(-1);
  }

  Vertices new_node_valences{0, 2};
  graph.processNewNode(gtsam::Symbol('a', 0),
                       gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2)),
                       false);
  graph.processNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');

  // Add node measurement
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> measurements;
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2))));

  graph.processNodeMeasurements(measurements);

  auto factors = graph.getFactors();

  EXPECT_EQ(11u, factors->size());
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(10)));

  OptimizeOnce(graph);

  pcl::PolygonMesh new_mesh =
      graph.deformMesh(simple_mesh, simple_mesh_stamps, simple_mesh_inds, 'v', 1);
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

  factors = graph.getFactors();
  EXPECT_EQ(10u, factors->size());

  // Add another prior to see if mesh reset
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> priors;
  priors.push_back({gtsam::Symbol('a', 0),
                    gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2))});
  graph.processNodeMeasurements(priors);

  OptimizeOnce(graph);

  new_mesh =
      graph.deformMesh(simple_mesh, simple_mesh_stamps, simple_mesh_inds, 'v', 1);
  pcl::fromPCLPointCloud2(new_mesh.cloud, actual_vertices);

  EXPECT_NEAR(0, actual_vertices.points[0].x, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[0].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[0].z, 0.001);
  EXPECT_NEAR(1, actual_vertices.points[1].x, 0.001);
  EXPECT_NEAR(1, actual_vertices.points[2].y, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[1].z, 0.001);
  EXPECT_NEAR(0, actual_vertices.points[2].x, 0.001);
}

TEST(TestDeformationGraph, processNewBetween) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh original_mesh;
  std::vector<Timestamp> original_mesh_stamps;
  std::vector<int> original_mesh_inds;
  SetUpOriginalMesh(original_mesh, &original_mesh_stamps, &original_mesh_inds);

  Vertices new_node_valences{0, 2};
  graph.processNewNode(
      gtsam::Symbol('a', 0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), true);
  graph.processNewNode(gtsam::Symbol('a', 1),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)),
                       false);
  graph.processNewBetween(gtsam::Symbol('a', 0),
                          gtsam::Symbol('a', 1),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)));
  graph.processNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');

  OptimizeOnce(graph);

  auto factors = graph.getFactors();
  auto values = graph.getValues();
  ASSERT_EQ(12u, factors->size());
  ASSERT_EQ(5u, values->size());

  // Check the prior factor
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(6)));
  gtsam::PriorFactor<gtsam::Pose3> prior_factor =
      *cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(6));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                                  prior_factor.prior()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), prior_factor.key());

  // Check the between factor
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(7)));
  gtsam::BetweenFactor<gtsam::Pose3> odom_factor =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(7));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)),
                                  odom_factor.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), odom_factor.front());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), odom_factor.back());

  std::vector<gtsam::Pose3> traj = graph.getTrajectory('a');
  EXPECT_EQ(2, traj.size());
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                                  traj[0]));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)),
                                  traj[1]));

  graph.processNewNode(gtsam::Symbol('a', 2),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)),
                       false);
  graph.processNewBetween(gtsam::Symbol('a', 1),
                          gtsam::Symbol('a', 2),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)));
  graph.processNewBetween(gtsam::Symbol('a', 0),
                          gtsam::Symbol('a', 2),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));

  Vertices new_node_valences_2{2};
  graph.processNodeValence(gtsam::Symbol('a', 2), new_node_valences_2, 'v');

  OptimizeOnce(graph);

  ASSERT_EQ(16u, factors->size());
  ASSERT_EQ(6u, values->size());

  // Check the between factor
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(12)));
  gtsam::BetweenFactor<gtsam::Pose3> new1 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(12));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)), new1.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), new1.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), new1.back());

  // Check the between factor
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(13)));
  gtsam::BetweenFactor<gtsam::Pose3> new2 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(13));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                  new2.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), new2.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), new2.back());

  traj = graph.getTrajectory('a');
  ASSERT_EQ(3, traj.size());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.05, 2.05)), traj[2], 0.05));
}

TEST(TestDeformationGraph, addTemporary) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  Vertices new_node_valences{0, 2};
  graph.processNewNode(
      gtsam::Symbol('a', 0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), true);
  graph.processNewNode(gtsam::Symbol('a', 1),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)),
                       false);
  graph.processNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');
  graph.processNewBetween(gtsam::Symbol('a', 0),
                          gtsam::Symbol('a', 1),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)));

  graph.processNewNode(gtsam::Symbol('a', 2),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)),
                       false);
  graph.processNewBetween(gtsam::Symbol('a', 1),
                          gtsam::Symbol('a', 2),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)));

  Vertices new_node_valences_2{2};
  graph.processNodeValence(gtsam::Symbol('a', 2), new_node_valences_2, 'v');

  // Check added factors
  auto factors = graph.getFactors();
  auto values = graph.getValues();
  auto temp_factors = graph.getTempFactors();
  auto temp_values = graph.getTempValues();

  EXPECT_EQ(15u, factors->size());
  EXPECT_EQ(6u, values->size());

  OptimizeOnce(graph);

  std::vector<gtsam::Pose3> traj = graph.getTrajectory('a');
  EXPECT_EQ(3, traj.size());

  // Add temporary nodes and edges
  graph.processNewTempNode(gtsam::Symbol('p', 0), gtsam::Pose3(), false);
  graph.processNewTempNode(gtsam::Symbol('p', 1),
                           gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)),
                           false);
  graph.processNewTempBetween(gtsam::Symbol('p', 0),
                              gtsam::Symbol('p', 1),
                              gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)));
  Vertices temp_node_valences{1, 2};
  graph.processNodeValence(gtsam::Symbol('p', 0), temp_node_valences, 'v', 1e-4, true);

  // Check added factors
  OptimizeOnce(graph);

  // Check added factors
  EXPECT_EQ(15u, factors->size());
  EXPECT_EQ(6u, values->size());

  EXPECT_EQ(5u, temp_factors->size());
  EXPECT_EQ(2u, temp_values->size());

  traj = graph.getTrajectory('a');
  EXPECT_EQ(3, traj.size());

  graph.clearTemporaryStructures();

  OptimizeOnce(graph);

  EXPECT_EQ(0u, temp_factors->size());
  EXPECT_EQ(0u, temp_values->size());

  // Re-ad temporary nodes and edges
  graph.processNewTempNode(gtsam::Symbol('p', 0), gtsam::Pose3(), false);
  Vertices temp_node_valences_2{0, 1, 2};
  graph.processNodeValence(
      gtsam::Symbol('p', 0), temp_node_valences_2, 'v', 1e-4, true);

  OptimizeOnce(graph);

  EXPECT_EQ(6u, temp_factors->size());
  EXPECT_EQ(1u, temp_values->size());
}

TEST(TestDeformationGraph, processNewTempNodesValences) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  NodeValenceInfoList info;
  for (size_t i = 0; i < 4; i++) {
    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0));
    info.push_back({'v', gtsam::Symbol('p', i), pose, {i}});
  }

  graph.processNewTempNodesValences(info, false);

  OptimizeOnce(graph);
  auto factors = graph.getFactors();
  auto values = graph.getValues();
  auto temp_factors = graph.getTempFactors();
  auto temp_values = graph.getTempValues();

  EXPECT_EQ(6u, factors->size());
  EXPECT_EQ(3u, values->size());

  for (size_t i = 0; i < 3; i++) {
    gtsam::Pose3 node_pose(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0));
    gtsam::Pose3 vertex_pose = values->at<gtsam::Pose3>(gtsam::Symbol('v', i));
    EXPECT_TRUE(gtsam::assert_equal(
        node_pose, temp_values->at<gtsam::Pose3>(gtsam::Symbol('p', i))));
    DeformationEdgeFactor factor1 =
        *cast_factor<DeformationEdgeFactor>(temp_factors->at(2 * i));
    DeformationEdgeFactor factor2 =
        *cast_factor<DeformationEdgeFactor>(temp_factors->at(2 * i + 1));
    EXPECT_TRUE(gtsam::assert_equal(
        node_pose.rotation().inverse().rotate(vertex_pose.translation() -
                                              node_pose.translation()),
        factor1.measurement()));
    EXPECT_TRUE(gtsam::assert_equal(
        vertex_pose.rotation().inverse().rotate(node_pose.translation() -
                                                vertex_pose.translation()),
        factor2.measurement()));
    EXPECT_EQ(gtsam::Symbol('p', i).key(), factor1.front());
    EXPECT_EQ(gtsam::Symbol('v', i).key(), factor1.back());
    EXPECT_EQ(gtsam::Symbol('v', i).key(), factor2.front());
    EXPECT_EQ(gtsam::Symbol('p', i).key(), factor2.back());
  }

  graph.clearTemporaryStructures();
  OptimizeOnce(graph);

  EXPECT_EQ(0u, temp_factors->size());
  EXPECT_EQ(0u, temp_values->size());
}

TEST(TestDeformationGraph, processNewTempEdges) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  NodeValenceInfoList info;
  for (size_t i = 0; i < 4; i++) {
    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0));
    info.push_back({'v', gtsam::Symbol('p', i), pose, {}});
  }

  graph.processNewTempNodesValences(info, false);

  pose_graph_tools::PoseGraph temp_edges;
  for (size_t i = 0; i < 3; i++) {
    pose_graph_tools::PoseGraphEdge edge;
    edge.key_from = gtsam::Symbol('p', i);
    edge.key_to = gtsam::Symbol('p', i + 1);
    temp_edges.edges.push_back(edge);
  }

  graph.processNewTempEdges(temp_edges);

  OptimizeOnce(graph);
  auto factors = graph.getFactors();
  auto values = graph.getValues();
  auto temp_factors = graph.getTempFactors();
  auto temp_values = graph.getTempValues();

  EXPECT_EQ(3u, values->size());
  EXPECT_EQ(6u, factors->size());
  EXPECT_EQ(4u, temp_values->size());
  EXPECT_EQ(3u, temp_factors->size());
}

TEST(TestDeformationGraph, saveAndLoad) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  Vertices new_node_valences{0, 2};
  graph.processNewNode(
      gtsam::Symbol('a', 0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), true);
  graph.processNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');

  graph.processNewNode(gtsam::Symbol('a', 1),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)),
                       false);
  graph.processNewBetween(gtsam::Symbol('a', 0),
                          gtsam::Symbol('a', 1),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)));

  graph.processNewNode(gtsam::Symbol('a', 2),
                       gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)),
                       false);
  graph.processNewBetween(gtsam::Symbol('a', 1),
                          gtsam::Symbol('a', 2),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)));

  Vertices new_node_valences_2{2};
  graph.processNodeValence(gtsam::Symbol('a', 2), new_node_valences_2, 'v');

  // Add temporary nodes and edges
  graph.processNewTempNode(gtsam::Symbol('p', 0), gtsam::Pose3(), false);
  graph.processNewTempNode(gtsam::Symbol('p', 1),
                           gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)),
                           false);
  graph.processNewTempBetween(gtsam::Symbol('p', 0),
                              gtsam::Symbol('p', 1),
                              gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)));
  Vertices temp_node_valences{1, 2};
  graph.processNodeValence(gtsam::Symbol('p', 0), temp_node_valences, 'v', 1e-4, true);

  auto values = graph.getValues();
  auto factors = graph.getFactors();
  auto temp_values = graph.getTempValues();
  auto temp_factors = graph.getTempFactors();

  EXPECT_EQ(15u, factors->size());
  EXPECT_EQ(6u, values->size());

  EXPECT_EQ(5u, temp_factors->size());
  EXPECT_EQ(2u, temp_values->size());
  graph.save(std::string(DATASET_PATH) + "/graph.dgrf");
  DeformationGraph new_graph;
  new_graph.load(std::string(DATASET_PATH) + "/graph.dgrf");

  values = new_graph.getValues();
  factors = new_graph.getFactors();
  temp_values = new_graph.getTempValues();
  temp_factors = new_graph.getTempFactors();

  EXPECT_EQ(15u, factors->size());
  EXPECT_EQ(6u, values->size());

  EXPECT_EQ(5u, temp_factors->size());
  EXPECT_EQ(2u, temp_values->size());
  EXPECT_EQ(3, new_graph.getNumVertices());
  EXPECT_EQ(0, new_graph.getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, new_graph.getInitialPositionVertex('v', 2).y());
}

TEST(TestDeformationGraph, processPoseGraph) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph, false);

  // Construct pose graph
  pose_graph_tools::PoseGraph pose_graph;
  size_t num_poses = 5;
  for (size_t i = 0; i < num_poses; i++) {
    pose_graph_tools::PoseGraphNode pg_node;
    pg_node.robot_id = 0;
    pg_node.key = i;
    pg_node.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(i, 0, 0)).matrix();
    pose_graph.nodes.push_back(pg_node);

    if (i == 0) {
      continue;
    }

    pose_graph_tools::PoseGraphEdge pg_edge;
    pg_edge.robot_from = 0;
    pg_edge.robot_to = 0;
    pg_edge.key_from = i - 1;
    pg_edge.key_to = i;
    pg_edge.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)).matrix();
    pg_edge.type = pose_graph_tools::PoseGraphEdge::Type::ODOM;
    pose_graph.edges.push_back(pg_edge);
  }

  // Create variance mapping
  std::map<pose_graph_tools::PoseGraphEdge::Type, double> variance_map;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::ODOM] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::LOOPCLOSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::PRIOR] = 0.01;

  // Process pose graph
  graph.processPoseGraph(pose_graph, variance_map);

  auto values = graph.getValues();
  auto factors = graph.getFactors();
  EXPECT_EQ(num_poses - 1, factors->size());
  EXPECT_EQ(num_poses, values->size());
}

TEST(TestDeformationGraph, processMeshGraph) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  // Construct mesh graph
  std::map<size_t, std::vector<Timestamp>> timestamps;
  auto mesh_graph = graph.getPoseGraph(timestamps, true, false);

  // Create variance mapping
  std::map<pose_graph_tools::PoseGraphEdge::Type, double> variance_map;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::MESH_POSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::POSE_MESH] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::MESH] = 0.01;

  // Process pose graph
  DeformationGraph new_graph;
  SetUpDeformationGraph(&new_graph, false);
  new_graph.processMeshGraph(*mesh_graph, variance_map);

  auto values = graph.getValues();
  auto factors = graph.getFactors();
  auto new_values = new_graph.getValues();
  auto new_factors = new_graph.getFactors();
  EXPECT_EQ(new_values->size(), values->size());
  EXPECT_EQ(new_factors->size(), factors->size());
  for (const auto& key_value : *values) {
    EXPECT_EQ(values->at<gtsam::Pose3>(key_value.key).matrix(),
              new_values->at<gtsam::Pose3>(key_value.key).matrix());
  }
}
TEST(TestDeformationGraph, processPoseMeshGraph) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);
  pcl::PolygonMesh original_mesh;
  std::vector<Timestamp> original_mesh_stamps;
  std::vector<int> original_mesh_inds;
  SetUpOriginalMesh(original_mesh, &original_mesh_stamps, &original_mesh_inds);

  Vertices new_node_valences{0, 2};
  graph.processNewNode(
      gtsam::Symbol('a', 0), gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), true);
  graph.processNodeValence(gtsam::Symbol('a', 0), new_node_valences, 'v');
  graph.processNewBetween(gtsam::Symbol('a', 0),
                          gtsam::Symbol('a', 1),
                          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)));

  // Construct mesh graph
  std::map<size_t, std::vector<Timestamp>> timestamps;
  timestamps[0] = {static_cast<uint64_t>(1e+9), static_cast<uint64_t>(2e+9)};
  auto pose_graph = graph.getPoseGraph(timestamps, false, true);
  auto mesh_graph = graph.getPoseGraph(timestamps, true, false);

  // Create variance mapping
  std::map<pose_graph_tools::PoseGraphEdge::Type, double> variance_map;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::LOOPCLOSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::REJECTED_LOOPCLOSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::ODOM] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::PRIOR] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::MESH_POSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::POSE_MESH] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::MESH] = 0.01;

  // Process pose graph
  DeformationGraph new_graph;
  SetUpDeformationGraph(&new_graph, false);
  new_graph.processPoseGraph(*pose_graph, variance_map);
  new_graph.processMeshGraph(*mesh_graph, variance_map);

  auto values = graph.getValues();
  auto factors = graph.getFactors();
  auto new_values = new_graph.getValues();
  auto new_factors = new_graph.getFactors();
  EXPECT_EQ(new_values->size(), values->size());
  EXPECT_EQ(new_factors->size(), factors->size());
  for (const auto& key_value : *values) {
    EXPECT_EQ(values->at<gtsam::Pose3>(key_value.key).matrix(),
              new_values->at<gtsam::Pose3>(key_value.key).matrix());
  }
}
}  // namespace kimera_pgmo
