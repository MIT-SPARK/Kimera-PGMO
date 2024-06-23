/**
 * @file   test_mesh_frontend.cpp
 * @brief  Unit-tests for the graph structure
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera_pgmo/mesh_frontend_interface.h"
#include "pgmo_fixtures.h"

namespace kimera_pgmo {

using test::OrderedBlockMeshInterface;

namespace {

void CreateMeshBlock(test::MeshLayer& mesh,
                     const BlockIndex& index,
                     const std::vector<float>& x_coords,
                     const std::vector<float>& y_coords,
                     const std::vector<float>& z_coords) {
  auto& block = mesh.allocateBlock(index);
  for (size_t i = 0; i < x_coords.size(); i++) {
    pcl::PointXYZRGBA point;
    point.x = x_coords[i];
    point.y = y_coords[i];
    point.z = z_coords[i];
    point.r = 123;
    point.g = 22;
    point.b = 250;
    point.a = 255;
    block.vertices.push_back(point);
  }
}

// time = 10.0
OrderedBlockMeshInterface CreateSimpleMesh1() {
  const auto mesh = std::make_shared<test::MeshLayer>(1.6);

  // single mesh block
  std::vector<float> x_coords = {0, 1, 0, 1, 1, 0};
  std::vector<float> y_coords = {0, 0, 1, 0, 1, 1};
  std::vector<float> z_coords = {0, 0, 0, 0, 0, 0};
  CreateMeshBlock(*mesh, BlockIndex(0, 0, 0), x_coords, y_coords, z_coords);

  OrderedBlockMeshInterface interface(mesh, {{0, 0, 0}});
  return interface;
}

// time = 10.5
OrderedBlockMeshInterface CreateSimpleMesh2() {
  const auto mesh = std::make_shared<test::MeshLayer>(1.6);

  std::vector<float> x_coords = {0, 1, 0, 1, 1, 0};
  std::vector<float> y_coords = {0, 0, 1, 0, 1, 1};
  std::vector<float> z_coords = {0, 0, 0, 0, 0, 0};
  CreateMeshBlock(*mesh, BlockIndex(0, 0, 0), x_coords, y_coords, z_coords);

  x_coords = {2, 3, 2, 3, 3, 2};
  y_coords = {0, 0, 1, 0, 1, 1};
  z_coords = {0, 0, 0, 0, 0, 0};
  CreateMeshBlock(*mesh, BlockIndex(1, 0, 0), x_coords, y_coords, z_coords);

  x_coords = {2, 3, 2, 3, 3, 2};
  y_coords = {2, 2, 3, 2, 3, 3};
  z_coords = {0, 0, 0, 0, 0, 0};
  CreateMeshBlock(*mesh, BlockIndex(1, 1, 0), x_coords, y_coords, z_coords);

  return {mesh, {{0, 0, 0}, {1, 0, 0}, {1, 1, 0}}};
}

// time = 11.6
test::OrderedBlockMeshInterface CreateSimpleMesh3() {
  const auto mesh = std::make_shared<test::MeshLayer>(1.6);

  std::vector<float> x_coords = {0, 1, 0, 1, 1, 0};
  std::vector<float> y_coords = {0, 0, 1, 0, 1, 1};
  std::vector<float> z_coords = {0, 0, 0, 0, 0, 0};
  CreateMeshBlock(*mesh, BlockIndex(0, 0, 0), x_coords, y_coords, z_coords);

  x_coords = {2, 3, 2, 3, 3, 2};
  y_coords = {0, 0, 1, 0, 1, 1};
  z_coords = {2, 2, 2, 2, 2, 2};
  CreateMeshBlock(*mesh, BlockIndex(1, 0, 1), x_coords, y_coords, z_coords);

  return {mesh, {{0, 0, 0}, {1, 0, 1}}};
}

// time = 10.5
OrderedBlockMeshInterface CreateSimpleMesh4() {
  const auto mesh = std::make_shared<test::MeshLayer>(1.6);

  std::vector<float> x_coords = {3.5, 4.5, 3.5, 4.5, 4.5, 3.5};
  std::vector<float> y_coords = {3.5, 3.5, 4.5, 3.5, 4.5, 4.5};
  std::vector<float> z_coords = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  CreateMeshBlock(*mesh, BlockIndex(2, 2, 0), x_coords, y_coords, z_coords);

  x_coords = {3.5, 4.5, 3.5, 4.5, 4.5, 3.5};
  y_coords = {3.5, 3.5, 4.5, 3.5, 4.5, 4.5};
  z_coords = {3.5, 4.5, 3.5, 4.5, 3.5, 3.5};
  CreateMeshBlock(*mesh, BlockIndex(2, 2, 2), x_coords, y_coords, z_coords);

  return {mesh, {{2, 2, 0}, {2, 2, 2}}};
}

// time = 10.5
OrderedBlockMeshInterface CreateSimpleMesh5() {
  const auto mesh = std::make_shared<test::MeshLayer>(1.6);

  std::vector<float> x_coords = {1.7, 3.1, 3.1, 1.7, 1.7, 3.1};
  std::vector<float> y_coords = {0.0, 0.0, 1.5, 0.0, 1.5, 1.5};
  std::vector<float> z_coords = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  CreateMeshBlock(*mesh, BlockIndex(1, 0, 0), x_coords, y_coords, z_coords);

  x_coords = {1.7, 3.1, 3.1, 1.7, 1.7, 3.1};
  y_coords = {1.7, 1.7, 3.1, 1.7, 3.1, 3.1};
  z_coords = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
  CreateMeshBlock(*mesh, BlockIndex(1, 1, 0), x_coords, y_coords, z_coords);

  return {mesh, {{1, 0, 0}, {1, 1, 0}}};
}

}  // namespace

class MeshFrontendTest : public ::testing::Test {
 protected:
  struct MeshResult {
    using Vertices = pcl::PointCloud<pcl::PointXYZRGBA>;
    using Faces = std::vector<pcl::Vertices>;
    using MeshPair = std::pair<Vertices::Ptr, Faces>;
    bool valid;
    MeshPair full;
    MeshPair graph;
    pose_graph_tools::PoseGraph deformation_graph;
    HashedIndexMapping mapping;

    MeshResult() : valid(false) {
      full = MeshPair(Vertices::Ptr(new Vertices()), Faces());
      graph = MeshPair(Vertices::Ptr(new Vertices()), Faces());
    }

    operator bool() const { return valid; }
  };

  MeshFrontendTest() {
    config.robot_id = 0;
    config.time_horizon = 1.0;
    config.full_compression_method = MeshFrontendInterface::FullCompressionMethod::OCTREE;
    config.graph_compression_method = MeshFrontendInterface::GraphCompressionMethod::OCTREE;
    config.d_graph_resolution = 0.5;
    config.mesh_resolution = 0.05;
  }

  ~MeshFrontendTest() {}

  void init() { frontend = std::make_shared<MeshFrontendInterface>(config); }

  MeshResult update(const OrderedBlockMeshInterface& mesh, double time_in_sec) {
    if (!frontend) {
      return {};
    }

    frontend->update(mesh, time_in_sec);

    MeshResult result;
    result.valid = true;
    *result.full.first = *frontend->vertices_;
    result.full.second = *frontend->triangles_;
    *result.graph.first = *frontend->graph_vertices_;
    result.graph.second = *frontend->graph_triangles_;
    result.deformation_graph = *frontend->last_mesh_graph_;
    result.mapping = *frontend->msg_to_graph_idx_;
    return result;
  }

  MeshFrontendInterface::Config config;
  std::shared_ptr<MeshFrontendInterface> frontend;
};

TEST_F(MeshFrontendTest, initialize) {
  config.mesh_resolution = 0.1;
  try {
    MeshFrontendInterface f(config);
    SUCCEED();
  } catch (const std::exception&) {
    FAIL();
  }
}

TEST_F(MeshFrontendTest, simplifiedMeshHighRes) {
  // Test the construction of the simplified mesh during callback
  // Case where graph resolution is higher than input mesh
  config.mesh_resolution = 0.1;
  init();
  ASSERT_TRUE(frontend);

  {  // add first mesh
    const auto mesh = CreateSimpleMesh1();
    const auto result = update(mesh, 10.0);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.graph;

    ASSERT_EQ(4u, vertices->points.size());
    ASSERT_EQ(2u, triangles.size());
    // Test points and triangles
    EXPECT_NEAR(1, vertices->points[1].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[3].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[3].y, 1e-3);
    EXPECT_EQ(2, triangles[0].vertices[2]);
    EXPECT_EQ(1, triangles[1].vertices[0]);
  }

  {  // Add second mesh
    const auto mesh = CreateSimpleMesh2();
    const auto result = update(mesh, 10.5);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.graph;

    ASSERT_EQ(12u, vertices->points.size());
    ASSERT_EQ(6u, triangles.size());
    // Test points and triangles
    EXPECT_NEAR(3, vertices->points[5].x, 1e-3);
    EXPECT_NEAR(3, vertices->points[7].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[7].y, 1e-3);
    EXPECT_EQ(6, triangles[2].vertices[2]);
    EXPECT_EQ(5, triangles[3].vertices[0]);

    EXPECT_NEAR(2, vertices->points[9].y, 1e-3);
    EXPECT_NEAR(3, vertices->points[11].x, 1e-3);
    EXPECT_NEAR(3, vertices->points[11].y, 1e-3);
    EXPECT_EQ(10, triangles[4].vertices[2]);
  }

  {  // This last mesh has stamp 11.6 so last mesh is not within time horizon
    // meaning that for duplicated blocks, won't check if points duplicated
    const auto mesh3 = CreateSimpleMesh3();
    const auto result = update(mesh3, 11.6);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.graph;

    ASSERT_EQ(20u, vertices->points.size());
    ASSERT_EQ(10u, triangles.size());
    // Test points and triangles
    EXPECT_NEAR(1, vertices->points[13].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[15].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[15].y, 1e-3);
    EXPECT_EQ(14, triangles[6].vertices[2]);
    EXPECT_EQ(13, triangles[7].vertices[0]);

    EXPECT_NEAR(3, vertices->points[17].x, 1e-3);
    EXPECT_NEAR(2, vertices->points[19].z, 1e-3);
    EXPECT_NEAR(1, vertices->points[19].y, 1e-3);
    EXPECT_EQ(18, triangles[8].vertices[2]);
    EXPECT_EQ(17, triangles[9].vertices[0]);
  }
}

TEST_F(MeshFrontendTest, simplifiedMeshLowRes) {
  // Test the construction of the simplified mesh during callback
  // Case where graph resolution is lower than input mesh
  config.mesh_resolution = 0.1;
  config.d_graph_resolution = 1.5;
  init();
  ASSERT_TRUE(frontend);

  {
    const auto mesh = CreateSimpleMesh1();
    const auto result = update(mesh, 10.0);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.graph;

    EXPECT_EQ(0u, vertices->points.size());
    EXPECT_EQ(0u, triangles.size());
  }

  {  // Add second mesh
    const auto mesh = CreateSimpleMesh2();
    const auto result = update(mesh, 10.5);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.graph;

    EXPECT_EQ(0u, vertices->points.size());
    EXPECT_EQ(0u, triangles.size());
  }

  {  // This last mesh has stamp 11.6 so last mesh is not within time horizon
     // meaning that for duplicated blocks, won't check if points duplicated
    const auto mesh = CreateSimpleMesh3();
    const auto result = update(mesh, 11.6);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.graph;

    EXPECT_EQ(0u, vertices->points.size());
    EXPECT_EQ(0u, triangles.size());
  }
}

TEST_F(MeshFrontendTest, fullMesh) {
  config.mesh_resolution = 0.1;
  init();
  ASSERT_TRUE(frontend);

  {
    const auto mesh = CreateSimpleMesh1();
    const auto result = update(mesh, 10.0);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.full;

    EXPECT_EQ(4u, vertices->points.size());
    EXPECT_EQ(2u, triangles.size());
    // Test points and triangles
    EXPECT_NEAR(1, vertices->points[1].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[3].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[3].y, 1e-3);
    EXPECT_EQ(2, triangles[0].vertices[2]);
    EXPECT_EQ(1, triangles[1].vertices[0]);
  }

  {  // Add second mesh
    const auto mesh = CreateSimpleMesh2();
    const auto result = update(mesh, 10.5);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.full;

    EXPECT_EQ(12u, vertices->points.size());
    EXPECT_EQ(6u, triangles.size());
    // Test points and triangles
    EXPECT_NEAR(3, vertices->points[5].x, 1e-3);
    EXPECT_NEAR(3, vertices->points[7].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[7].y, 1e-3);
    EXPECT_EQ(6, triangles[2].vertices[2]);
    EXPECT_EQ(5, triangles[3].vertices[0]);

    EXPECT_NEAR(2, vertices->points[9].y, 1e-3);
    EXPECT_NEAR(3, vertices->points[11].x, 1e-3);
    EXPECT_NEAR(3, vertices->points[11].y, 1e-3);
    EXPECT_EQ(10, triangles[4].vertices[2]);
  }

  {  // This last mesh has stamp 11.6 so last mesh is not within time horizon
    // meaning that for duplicated blocks, won't check if points duplicated
    const auto mesh = CreateSimpleMesh3();
    const auto result = update(mesh, 11.6);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.full;

    EXPECT_EQ(20u, vertices->points.size());
    EXPECT_EQ(10u, triangles.size());
    // Test points and triangles
    EXPECT_NEAR(1, vertices->points[13].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[15].x, 1e-3);
    EXPECT_NEAR(1, vertices->points[15].y, 1e-3);
    EXPECT_EQ(14, triangles[6].vertices[2]);
    EXPECT_EQ(13, triangles[7].vertices[0]);

    EXPECT_NEAR(3, vertices->points[17].x, 1e-3);
    EXPECT_NEAR(2, vertices->points[19].z, 1e-3);
    EXPECT_NEAR(1, vertices->points[19].y, 1e-3);
    EXPECT_EQ(18, triangles[8].vertices[2]);
    EXPECT_EQ(17, triangles[9].vertices[0]);
  }
}

TEST_F(MeshFrontendTest, compression1) {
  // Test with higher resolution to see if compression works
  config.mesh_resolution = 4.0;
  init();
  ASSERT_TRUE(frontend);

  {
    const auto mesh = CreateSimpleMesh1();
    const auto result = update(mesh, 10.0);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.full;

    EXPECT_EQ(0u, vertices->points.size());
    EXPECT_EQ(0u, triangles.size());
  }

  {  // Add second mesh
    const auto mesh = CreateSimpleMesh2();
    const auto result = update(mesh, 10.5);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.full;

    EXPECT_EQ(0u, vertices->points.size());
    EXPECT_EQ(0u, triangles.size());
  }

  {  // This last mesh has stamp 11.6 so last mesh is not within time horizon
    // meaning that for duplicated blocks, won't check if points duplicated
    const auto mesh = CreateSimpleMesh3();
    const auto result = update(mesh, 11.6);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.full;

    EXPECT_EQ(0u, vertices->points.size());
    EXPECT_EQ(0u, triangles.size());
  }
}

TEST_F(MeshFrontendTest, compression2) {
  // Test with higher resolution to see if compression works
  config.mesh_resolution = 4.0;
  init();
  ASSERT_TRUE(frontend);

  {
    const auto mesh = CreateSimpleMesh1();
    const auto result = update(mesh, 10.0);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.full;

    EXPECT_EQ(0u, vertices->points.size());
    EXPECT_EQ(0u, triangles.size());
  }

  {  // process another mesh
    const auto mesh = CreateSimpleMesh4();
    const auto result = update(mesh, 10.5);
    ASSERT_TRUE(result);
    const auto& [vertices, triangles] = result.full;

    EXPECT_EQ(0u, vertices->points.size());
    EXPECT_EQ(0u, triangles.size());
  }
}

TEST_F(MeshFrontendTest, meshGraph) {
  // Test with higher resolution to see if compression works
  config.mesh_resolution = 4.0;
  config.d_graph_resolution = 0.6;
  init();
  ASSERT_TRUE(frontend);

  {
    const auto mesh = CreateSimpleMesh1();
    const auto result = update(mesh, 10.0);
    ASSERT_TRUE(result);
    const auto& last_graph = result.deformation_graph;

    // Check size
    ASSERT_EQ(4, last_graph.nodes.size());
    ASSERT_EQ(10, last_graph.edges.size());

    // Check first and last edge
    EXPECT_EQ(0, last_graph.edges[0].robot_from);
    EXPECT_EQ(0, last_graph.edges[0].robot_to);
    EXPECT_EQ(0, last_graph.edges[0].key_from);
    EXPECT_EQ(1, last_graph.edges[0].key_to);
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                    gtsam::Pose3(last_graph.edges[0].pose.matrix()),
                                    1e-4));

    EXPECT_EQ(0, last_graph.edges[9].robot_from);
    EXPECT_EQ(0, last_graph.edges[9].robot_to);
    EXPECT_EQ(2, last_graph.edges[9].key_from);
    EXPECT_EQ(3, last_graph.edges[9].key_to);
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                    gtsam::Pose3(last_graph.edges[9].pose.matrix()),
                                    1e-4));

    // Check first and last node
    EXPECT_EQ(0, last_graph.nodes[0].robot_id);
    EXPECT_EQ(0, last_graph.nodes[0].key);
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                                    gtsam::Pose3(last_graph.nodes[0].pose.matrix()),
                                    1e-4));

    EXPECT_EQ(0, last_graph.nodes[3].robot_id);
    EXPECT_EQ(3, last_graph.nodes[3].key);
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                    gtsam::Pose3(last_graph.nodes[3].pose.matrix()),
                                    1e-4));
  }

  {  // process another mesh
    const auto mesh = CreateSimpleMesh4();
    const auto result = update(mesh, 10.5);
    ASSERT_TRUE(result);
    const auto& last_graph = result.deformation_graph;

    // Check size
    EXPECT_EQ(8, last_graph.nodes.size());
    EXPECT_EQ(20, last_graph.edges.size());

    // Check first and last edge
    EXPECT_EQ(0, last_graph.edges[0].robot_from);
    EXPECT_EQ(0, last_graph.edges[0].robot_to);
    EXPECT_EQ(4, last_graph.edges[0].key_from);
    EXPECT_EQ(5, last_graph.edges[0].key_to);
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                    gtsam::Pose3(last_graph.edges[0].pose.matrix()),
                                    1e-4));

    EXPECT_EQ(0, last_graph.edges[19].robot_from);
    EXPECT_EQ(0, last_graph.edges[19].robot_to);
    EXPECT_EQ(10, last_graph.edges[19].key_from);
    EXPECT_EQ(11, last_graph.edges[19].key_to);
    EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                    gtsam::Pose3(last_graph.edges[9].pose.matrix()),
                                    1e-4));

    // Check first and last node
    EXPECT_EQ(0, last_graph.nodes[0].robot_id);
    EXPECT_EQ(4, last_graph.nodes[0].key);
    EXPECT_TRUE(
        gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.5, 3.5, 0)),
                            gtsam::Pose3(last_graph.nodes[0].pose.matrix()),
                            1e-4));

    EXPECT_EQ(0, last_graph.nodes[7].robot_id);
    EXPECT_EQ(11, last_graph.nodes[7].key);
    EXPECT_TRUE(
        gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.5, 4.5, 3.5)),
                            gtsam::Pose3(last_graph.nodes[7].pose.matrix()),
                            1e-4));
  }
}

TEST_F(MeshFrontendTest, vxblxIndexMapping1) {
  // Test index mappings
  init();
  ASSERT_TRUE(frontend);

  const auto mesh1 = CreateSimpleMesh1();
  update(mesh1, 10.0);

  const auto mesh2 = CreateSimpleMesh5();
  const auto result = update(mesh2, 10.5);
  ASSERT_TRUE(result);

  const auto& mappings = result.mapping;

  BlockIndex block1(0, 0, 0);
  BlockIndex block2(1, 0, 0);
  BlockIndex block3(1, 1, 0);

  // Check the mappings
  EXPECT_EQ(3, mappings.size());
  EXPECT_EQ(6, mappings.at(block1).size());
  EXPECT_EQ(6, mappings.at(block2).size());
  EXPECT_EQ(6, mappings.at(block3).size());

  EXPECT_EQ(0, mappings.at(block1).at(0));
  EXPECT_EQ(1, mappings.at(block1).at(1));
  EXPECT_EQ(2, mappings.at(block1).at(2));
  EXPECT_EQ(1, mappings.at(block1).at(3));
  EXPECT_EQ(3, mappings.at(block1).at(4));
  EXPECT_EQ(2, mappings.at(block1).at(5));

  EXPECT_EQ(4, mappings.at(block2).at(0));
  EXPECT_EQ(5, mappings.at(block2).at(1));
  EXPECT_EQ(6, mappings.at(block2).at(2));
  EXPECT_EQ(4, mappings.at(block2).at(3));
  EXPECT_EQ(7, mappings.at(block2).at(4));
  EXPECT_EQ(6, mappings.at(block2).at(5));

  EXPECT_EQ(7, mappings.at(block3).at(0));
  EXPECT_EQ(6, mappings.at(block3).at(1));
  EXPECT_EQ(8, mappings.at(block3).at(2));
  EXPECT_EQ(7, mappings.at(block3).at(3));
  EXPECT_EQ(9, mappings.at(block3).at(4));
  EXPECT_EQ(8, mappings.at(block3).at(5));
}

TEST_F(MeshFrontendTest, vxblxIndexMapping2) {
  // Test index mappings
  config.mesh_resolution = 0.1;
  config.d_graph_resolution = 1.3;
  init();
  ASSERT_TRUE(frontend);

  const auto mesh1 = CreateSimpleMesh1();
  update(mesh1, 10.0);
  // process another mesh
  const auto mesh5 = CreateSimpleMesh5();
  const auto result = update(mesh5, 10.5);
  ASSERT_TRUE(result);
  const auto& mappings = result.mapping;

  BlockIndex block1(0, 0, 0);
  BlockIndex block2(1, 0, 0);
  BlockIndex block3(1, 1, 0);

  // Check the mappings
  EXPECT_EQ(3, mappings.size());
  EXPECT_EQ(0, mappings.at(block1).size());
  EXPECT_EQ(6, mappings.at(block2).size());
  EXPECT_EQ(6, mappings.at(block3).size());

  EXPECT_EQ(0, mappings.at(block2).at(0));
  EXPECT_EQ(1, mappings.at(block2).at(1));
  EXPECT_EQ(2, mappings.at(block2).at(2));
  EXPECT_EQ(0, mappings.at(block2).at(3));
  EXPECT_EQ(3, mappings.at(block2).at(4));
  EXPECT_EQ(2, mappings.at(block2).at(5));

  EXPECT_EQ(3, mappings.at(block3).at(0));
  EXPECT_EQ(2, mappings.at(block3).at(1));
  EXPECT_EQ(4, mappings.at(block3).at(2));
  EXPECT_EQ(3, mappings.at(block3).at(3));
  EXPECT_EQ(5, mappings.at(block3).at(4));
  EXPECT_EQ(4, mappings.at(block3).at(5));
}

}  // namespace kimera_pgmo
