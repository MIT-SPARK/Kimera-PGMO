/**
 * @file   test_mesh_frontend.cpp
 * @brief  Unit-tests for the graph structure
 * @author Yun Chang
 */
#include <ros/ros.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/MeshBlock.h>

#include "gtest/gtest.h"

#include "kimera_pgmo/MeshFrontend.h"
#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/VoxbloxUtils.h"

namespace kimera_pgmo {

class MeshFrontendTest : public ::testing::Test {
 protected:
  MeshFrontendTest() {
    system("rosparam set horizon 1.0");
    system("rosparam set robot_id 0");
    system("rosparam set d_graph_resolution 0.5");
  }

  ~MeshFrontendTest() {}

  // Test the created mesh graph
  inline pose_graph_tools::PoseGraph GetLastProcessedMeshGraph() const {
    return vp_.getLastProcessedMeshGraph();
  }

  // Test update called in timer event
  void ProcessVoxbloxMesh(const voxblox_msgs::Mesh::ConstPtr& msg) {
    vp_.processVoxbloxMesh(msg);
  }

  voxblox_msgs::MeshBlock CreateMeshBlock(
      const BlockIndex& index,
      const float& block_edge_length,
      const std::vector<float>& x_coords,
      const std::vector<float>& y_coords,
      const std::vector<float>& z_coords) const {
    voxblox_msgs::MeshBlock mesh_block;
    mesh_block.index[0] = index[0];
    mesh_block.index[1] = index[1];
    mesh_block.index[2] = index[2];
    constexpr float point_conv_factor =
        2.0f / std::numeric_limits<uint16_t>::max();
    for (size_t i = 0; i < x_coords.size(); i++) {
      mesh_block.x.push_back(static_cast<uint16_t>(
          (x_coords[i] / block_edge_length - static_cast<float>(index[0])) /
          point_conv_factor));
      mesh_block.y.push_back(static_cast<uint16_t>(
          (y_coords[i] / block_edge_length - static_cast<float>(index[1])) /
          point_conv_factor));
      mesh_block.z.push_back(static_cast<uint16_t>(
          (z_coords[i] / block_edge_length - static_cast<float>(index[2])) /
          point_conv_factor));
      mesh_block.r.push_back(123);
      mesh_block.g.push_back(22);
      mesh_block.b.push_back(250);
    }

    return mesh_block;
  }

  voxblox_msgs::Mesh CreateSimpleMesh1() const {
    // single mesh block
    std::vector<float> x_coords = {0, 1, 0, 1, 1, 0};
    std::vector<float> y_coords = {0, 0, 1, 0, 1, 1};
    std::vector<float> z_coords = {0, 0, 0, 0, 0, 0};

    voxblox_msgs::MeshBlock block =
        CreateMeshBlock(BlockIndex(0, 0, 0), 1.6, x_coords, y_coords, z_coords);

    voxblox_msgs::Mesh mesh1;
    mesh1.header.stamp = ros::Time(10.0);
    mesh1.block_edge_length = 1.6;
    mesh1.mesh_blocks.push_back(block);

    return mesh1;
  }

  voxblox_msgs::Mesh CreateSimpleMesh2() const {
    std::vector<float> x_coords = {0, 1, 0, 1, 1, 0};
    std::vector<float> y_coords = {0, 0, 1, 0, 1, 1};
    std::vector<float> z_coords = {0, 0, 0, 0, 0, 0};

    voxblox_msgs::MeshBlock block1 =
        CreateMeshBlock(BlockIndex(0, 0, 0), 1.6, x_coords, y_coords, z_coords);

    x_coords = {2, 3, 2, 3, 3, 2};
    y_coords = {0, 0, 1, 0, 1, 1};
    z_coords = {0, 0, 0, 0, 0, 0};

    voxblox_msgs::MeshBlock block2 =
        CreateMeshBlock(BlockIndex(1, 0, 0), 1.6, x_coords, y_coords, z_coords);

    x_coords = {2, 3, 2, 3, 3, 2};
    y_coords = {2, 2, 3, 2, 3, 3};
    z_coords = {0, 0, 0, 0, 0, 0};

    voxblox_msgs::MeshBlock block3 =
        CreateMeshBlock(BlockIndex(1, 1, 0), 1.6, x_coords, y_coords, z_coords);

    voxblox_msgs::Mesh mesh2;
    mesh2.header.stamp = ros::Time(10.5);
    mesh2.block_edge_length = 1.6;
    mesh2.mesh_blocks.push_back(block1);
    mesh2.mesh_blocks.push_back(block2);
    mesh2.mesh_blocks.push_back(block3);

    return mesh2;
  }

  voxblox_msgs::Mesh CreateSimpleMesh3() const {
    std::vector<float> x_coords = {0, 1, 0, 1, 1, 0};
    std::vector<float> y_coords = {0, 0, 1, 0, 1, 1};
    std::vector<float> z_coords = {0, 0, 0, 0, 0, 0};

    voxblox_msgs::MeshBlock block1 =
        CreateMeshBlock(BlockIndex(0, 0, 0), 1.6, x_coords, y_coords, z_coords);

    x_coords = {2, 3, 2, 3, 3, 2};
    y_coords = {0, 0, 1, 0, 1, 1};
    z_coords = {2, 2, 2, 2, 2, 2};

    voxblox_msgs::MeshBlock block2 =
        CreateMeshBlock(BlockIndex(1, 0, 1), 1.6, x_coords, y_coords, z_coords);

    voxblox_msgs::Mesh mesh3;
    mesh3.header.stamp = ros::Time(11.6);
    mesh3.block_edge_length = 1.6;
    mesh3.mesh_blocks.push_back(block1);
    mesh3.mesh_blocks.push_back(block2);

    return mesh3;
  }

  voxblox_msgs::Mesh CreateSimpleMesh4() const {
    std::vector<float> x_coords = {3.5, 4.5, 3.5, 4.5, 4.5, 3.5};
    std::vector<float> y_coords = {3.5, 3.5, 4.5, 3.5, 4.5, 4.5};
    std::vector<float> z_coords = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    voxblox_msgs::MeshBlock block1 =
        CreateMeshBlock(BlockIndex(2, 2, 0), 1.6, x_coords, y_coords, z_coords);

    x_coords = {3.5, 4.5, 3.5, 4.5, 4.5, 3.5};
    y_coords = {3.5, 3.5, 4.5, 3.5, 4.5, 4.5};
    z_coords = {3.5, 4.5, 3.5, 4.5, 3.5, 3.5};

    voxblox_msgs::MeshBlock block2 =
        CreateMeshBlock(BlockIndex(2, 2, 2), 1.6, x_coords, y_coords, z_coords);

    voxblox_msgs::Mesh mesh4;
    mesh4.header.stamp = ros::Time(10.5);
    mesh4.block_edge_length = 1.6;
    mesh4.mesh_blocks.push_back(block1);
    mesh4.mesh_blocks.push_back(block2);

    return mesh4;
  }

  void GetFullMesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices,
                   std::vector<pcl::Vertices>* triangles) {
    *vertices = *(vp_.vertices_);
    *triangles = *(vp_.triangles_);
  }

  void GetSimplifiedMesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices,
                         std::vector<pcl::Vertices>* triangles) {
    *vertices = *(vp_.graph_vertices_);
    *triangles = *(vp_.graph_triangles_);
  }

  MeshFrontend vp_;
};

TEST_F(MeshFrontendTest, initialize) {
  // Test initialization
  ros::NodeHandle nh;
  system("rosparam set output_mesh_resolution 0.1");
  bool init = vp_.initialize(nh);
  ASSERT_TRUE(init);
}

TEST_F(MeshFrontendTest, simplifiedMesh_high_res) {
  // Test the construction of the simplified mesh during callback
  // Case where graph resolution is higher than input mesh
  ros::NodeHandle nh;
  system("rosparam set output_mesh_resolution 0.1");
  vp_.initialize(nh);

  voxblox_msgs::Mesh::Ptr mesh1(new voxblox_msgs::Mesh);
  *mesh1 = CreateSimpleMesh1();

  ProcessVoxbloxMesh(mesh1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr simplified_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<pcl::Vertices> triangles;
  GetSimplifiedMesh(simplified_vertices, &triangles);

  EXPECT_EQ(size_t(4), simplified_vertices->points.size());
  EXPECT_EQ(size_t(2), triangles.size());
  // Test points and triangles
  EXPECT_NEAR(1, simplified_vertices->points[1].x, 1e-3);
  EXPECT_NEAR(1, simplified_vertices->points[3].x, 1e-3);
  EXPECT_NEAR(1, simplified_vertices->points[3].y, 1e-3);
  EXPECT_EQ(2, triangles[0].vertices[2]);
  EXPECT_EQ(1, triangles[1].vertices[0]);

  // Add second mesh
  voxblox_msgs::Mesh::Ptr mesh2(new voxblox_msgs::Mesh);
  *mesh2 = CreateSimpleMesh2();
  ProcessVoxbloxMesh(mesh2);
  GetSimplifiedMesh(simplified_vertices, &triangles);

  EXPECT_EQ(size_t(12), simplified_vertices->points.size());
  EXPECT_EQ(size_t(6), triangles.size());
  // Test points and triangles
  EXPECT_NEAR(3, simplified_vertices->points[5].x, 1e-3);
  EXPECT_NEAR(3, simplified_vertices->points[7].x, 1e-3);
  EXPECT_NEAR(1, simplified_vertices->points[7].y, 1e-3);
  EXPECT_EQ(6, triangles[2].vertices[2]);
  EXPECT_EQ(5, triangles[3].vertices[0]);

  EXPECT_NEAR(2, simplified_vertices->points[9].y, 1e-3);
  EXPECT_NEAR(3, simplified_vertices->points[11].x, 1e-3);
  EXPECT_NEAR(3, simplified_vertices->points[11].y, 1e-3);
  EXPECT_EQ(10, triangles[4].vertices[2]);

  // This last mesh has stamp 11.6 so last mesh is not within time horizon
  // meaning that for duplicated blocks, won't check if points duplicated
  voxblox_msgs::Mesh::Ptr mesh3(new voxblox_msgs::Mesh);
  *mesh3 = CreateSimpleMesh3();
  ProcessVoxbloxMesh(mesh3);
  GetSimplifiedMesh(simplified_vertices, &triangles);

  EXPECT_EQ(size_t(20), simplified_vertices->points.size());
  EXPECT_EQ(size_t(10), triangles.size());
  // Test points and triangles
  EXPECT_NEAR(1, simplified_vertices->points[13].x, 1e-3);
  EXPECT_NEAR(1, simplified_vertices->points[15].x, 1e-3);
  EXPECT_NEAR(1, simplified_vertices->points[15].y, 1e-3);
  EXPECT_EQ(14, triangles[6].vertices[2]);
  EXPECT_EQ(13, triangles[7].vertices[0]);

  EXPECT_NEAR(3, simplified_vertices->points[17].x, 1e-3);
  EXPECT_NEAR(2, simplified_vertices->points[19].z, 1e-3);
  EXPECT_NEAR(1, simplified_vertices->points[19].y, 1e-3);
  EXPECT_EQ(18, triangles[8].vertices[2]);
  EXPECT_EQ(17, triangles[9].vertices[0]);
}

TEST_F(MeshFrontendTest, simplifiedMesh_low_res) {
  // Test the construction of the simplified mesh during callback
  // Case where graph resolution is lower than input mesh
  ros::NodeHandle nh;
  system("rosparam set output_mesh_resolution 0.1");
  system("rosparam set d_graph_resolution 1.5");
  vp_.initialize(nh);

  voxblox_msgs::Mesh::Ptr mesh1(new voxblox_msgs::Mesh);
  *mesh1 = CreateSimpleMesh1();

  ProcessVoxbloxMesh(mesh1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr simplified_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<pcl::Vertices> triangles;
  GetSimplifiedMesh(simplified_vertices, &triangles);

  EXPECT_EQ(size_t(0), simplified_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles.size());

  // Add second mesh
  voxblox_msgs::Mesh::Ptr mesh2(new voxblox_msgs::Mesh);
  *mesh2 = CreateSimpleMesh2();
  ProcessVoxbloxMesh(mesh2);
  GetSimplifiedMesh(simplified_vertices, &triangles);

  EXPECT_EQ(size_t(0), simplified_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles.size());

  // This last mesh has stamp 11.6 so last mesh is not within time horizon
  // meaning that for duplicated blocks, won't check if points duplicated
  voxblox_msgs::Mesh::Ptr mesh3(new voxblox_msgs::Mesh);
  *mesh3 = CreateSimpleMesh3();
  ProcessVoxbloxMesh(mesh3);
  GetSimplifiedMesh(simplified_vertices, &triangles);

  EXPECT_EQ(size_t(0), simplified_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles.size());
}

TEST_F(MeshFrontendTest, fullMesh) {
  // Test the updated full mesh after callback
  ros::NodeHandle nh;
  system("rosparam set output_mesh_resolution 0.1");
  vp_.initialize(nh);

  voxblox_msgs::Mesh::Ptr mesh1(new voxblox_msgs::Mesh);
  *mesh1 = CreateSimpleMesh1();

  ProcessVoxbloxMesh(mesh1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr full_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<pcl::Vertices> triangles;
  GetFullMesh(full_vertices, &triangles);

  EXPECT_EQ(size_t(4), full_vertices->points.size());
  EXPECT_EQ(size_t(2), triangles.size());
  // Test points and triangles
  EXPECT_NEAR(1, full_vertices->points[1].x, 1e-3);
  EXPECT_NEAR(1, full_vertices->points[3].x, 1e-3);
  EXPECT_NEAR(1, full_vertices->points[3].y, 1e-3);
  EXPECT_EQ(2, triangles[0].vertices[2]);
  EXPECT_EQ(1, triangles[1].vertices[0]);

  // Add second mesh
  voxblox_msgs::Mesh::Ptr mesh2(new voxblox_msgs::Mesh);
  *mesh2 = CreateSimpleMesh2();
  ProcessVoxbloxMesh(mesh2);
  GetFullMesh(full_vertices, &triangles);

  EXPECT_EQ(size_t(12), full_vertices->points.size());
  EXPECT_EQ(size_t(6), triangles.size());
  // Test points and triangles
  EXPECT_NEAR(3, full_vertices->points[5].x, 1e-3);
  EXPECT_NEAR(3, full_vertices->points[7].x, 1e-3);
  EXPECT_NEAR(1, full_vertices->points[7].y, 1e-3);
  EXPECT_EQ(6, triangles[2].vertices[2]);
  EXPECT_EQ(5, triangles[3].vertices[0]);

  EXPECT_NEAR(2, full_vertices->points[9].y, 1e-3);
  EXPECT_NEAR(3, full_vertices->points[11].x, 1e-3);
  EXPECT_NEAR(3, full_vertices->points[11].y, 1e-3);
  EXPECT_EQ(10, triangles[4].vertices[2]);

  // This last mesh has stamp 11.6 so last mesh is not within time horizon
  // meaning that for duplicated blocks, won't check if points duplicated
  voxblox_msgs::Mesh::Ptr mesh3(new voxblox_msgs::Mesh);
  *mesh3 = CreateSimpleMesh3();
  ProcessVoxbloxMesh(mesh3);
  GetFullMesh(full_vertices, &triangles);

  EXPECT_EQ(size_t(20), full_vertices->points.size());
  EXPECT_EQ(size_t(10), triangles.size());
  // Test points and triangles
  EXPECT_NEAR(1, full_vertices->points[13].x, 1e-3);
  EXPECT_NEAR(1, full_vertices->points[15].x, 1e-3);
  EXPECT_NEAR(1, full_vertices->points[15].y, 1e-3);
  EXPECT_EQ(14, triangles[6].vertices[2]);
  EXPECT_EQ(13, triangles[7].vertices[0]);

  EXPECT_NEAR(3, full_vertices->points[17].x, 1e-3);
  EXPECT_NEAR(2, full_vertices->points[19].z, 1e-3);
  EXPECT_NEAR(1, full_vertices->points[19].y, 1e-3);
  EXPECT_EQ(18, triangles[8].vertices[2]);
  EXPECT_EQ(17, triangles[9].vertices[0]);
}

TEST_F(MeshFrontendTest, compression1) {
  // Test with higher resolution to see if compression works
  ros::NodeHandle nh;
  system("rosparam set output_mesh_resolution 4.0");
  vp_.initialize(nh);

  voxblox_msgs::Mesh::Ptr mesh1(new voxblox_msgs::Mesh);
  *mesh1 = CreateSimpleMesh1();

  ProcessVoxbloxMesh(mesh1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr full_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<pcl::Vertices> triangles;
  GetFullMesh(full_vertices, &triangles);

  EXPECT_EQ(size_t(0), full_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles.size());

  // Add second mesh
  voxblox_msgs::Mesh::Ptr mesh2(new voxblox_msgs::Mesh);
  *mesh2 = CreateSimpleMesh2();
  ProcessVoxbloxMesh(mesh2);
  GetFullMesh(full_vertices, &triangles);

  EXPECT_EQ(size_t(0), full_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles.size());

  // This last mesh has stamp 11.6 so last mesh is not within time horizon
  // meaning that for duplicated blocks, won't check if points duplicated
  voxblox_msgs::Mesh::Ptr mesh3(new voxblox_msgs::Mesh);
  *mesh3 = CreateSimpleMesh3();
  ProcessVoxbloxMesh(mesh3);
  GetFullMesh(full_vertices, &triangles);

  EXPECT_EQ(size_t(0), full_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles.size());
}

TEST_F(MeshFrontendTest, compression2) {
  // Test with higher resolution to see if compression works
  ros::NodeHandle nh;
  system("rosparam set output_mesh_resolution 4.0");
  vp_.initialize(nh);

  voxblox_msgs::Mesh::Ptr mesh1(new voxblox_msgs::Mesh);
  *mesh1 = CreateSimpleMesh1();

  ProcessVoxbloxMesh(mesh1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr full_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<pcl::Vertices> triangles;
  GetFullMesh(full_vertices, &triangles);

  EXPECT_EQ(size_t(0), full_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles.size());

  // process another mesh
  voxblox_msgs::Mesh::Ptr mesh4(new voxblox_msgs::Mesh);
  *mesh4 = CreateSimpleMesh4();
  ProcessVoxbloxMesh(mesh4);
  GetFullMesh(full_vertices, &triangles);

  EXPECT_EQ(size_t(0), full_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles.size());
}

TEST_F(MeshFrontendTest, meshGraph) {
  // Test with higher resolution to see if compression works
  ros::NodeHandle nh;
  system("rosparam set output_mesh_resolution 4.0");
  vp_.initialize(nh);

  voxblox_msgs::Mesh::Ptr mesh1(new voxblox_msgs::Mesh);
  *mesh1 = CreateSimpleMesh1();

  ProcessVoxbloxMesh(mesh1);

  pose_graph_tools::PoseGraph last_mesh_graph = GetLastProcessedMeshGraph();

  // Check size
  EXPECT_EQ(4, last_mesh_graph.nodes.size());
  EXPECT_EQ(10, last_mesh_graph.edges.size());

  // Check first and last edge
  EXPECT_EQ(0, last_mesh_graph.edges[0].robot_from);
  EXPECT_EQ(0, last_mesh_graph.edges[0].robot_to);
  EXPECT_EQ(0, last_mesh_graph.edges[0].key_from);
  EXPECT_EQ(1, last_mesh_graph.edges[0].key_to);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          RosToGtsam(last_mesh_graph.edges[0].pose),
                          1e-4));
  EXPECT_EQ(0, last_mesh_graph.edges[9].robot_from);
  EXPECT_EQ(0, last_mesh_graph.edges[9].robot_to);
  EXPECT_EQ(2, last_mesh_graph.edges[9].key_from);
  EXPECT_EQ(3, last_mesh_graph.edges[9].key_to);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          RosToGtsam(last_mesh_graph.edges[9].pose),
                          1e-4));

  // Check first and last node
  EXPECT_EQ(0, last_mesh_graph.nodes[0].robot_id);
  EXPECT_EQ(0, last_mesh_graph.nodes[0].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                          RosToGtsam(last_mesh_graph.nodes[0].pose),
                          1e-4));

  EXPECT_EQ(0, last_mesh_graph.nodes[3].robot_id);
  EXPECT_EQ(3, last_mesh_graph.nodes[3].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                          RosToGtsam(last_mesh_graph.nodes[3].pose),
                          1e-4));

  // process another mesh
  voxblox_msgs::Mesh::Ptr mesh4(new voxblox_msgs::Mesh);
  *mesh4 = CreateSimpleMesh4();
  ProcessVoxbloxMesh(mesh4);

  last_mesh_graph = GetLastProcessedMeshGraph();

  // Check size
  EXPECT_EQ(8, last_mesh_graph.nodes.size());
  EXPECT_EQ(20, last_mesh_graph.edges.size());

  // Check first and last edge
  EXPECT_EQ(0, last_mesh_graph.edges[0].robot_from);
  EXPECT_EQ(0, last_mesh_graph.edges[0].robot_to);
  EXPECT_EQ(4, last_mesh_graph.edges[0].key_from);
  EXPECT_EQ(5, last_mesh_graph.edges[0].key_to);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          RosToGtsam(last_mesh_graph.edges[0].pose),
                          1e-4));
  EXPECT_EQ(0, last_mesh_graph.edges[19].robot_from);
  EXPECT_EQ(0, last_mesh_graph.edges[19].robot_to);
  EXPECT_EQ(10, last_mesh_graph.edges[19].key_from);
  EXPECT_EQ(11, last_mesh_graph.edges[19].key_to);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          RosToGtsam(last_mesh_graph.edges[9].pose),
                          1e-4));

  // Check first and last node
  EXPECT_EQ(0, last_mesh_graph.nodes[0].robot_id);
  EXPECT_EQ(4, last_mesh_graph.nodes[0].key);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3.5, 3.5, 0)),
      RosToGtsam(last_mesh_graph.nodes[0].pose),
      1e-4));

  EXPECT_EQ(0, last_mesh_graph.nodes[7].robot_id);
  EXPECT_EQ(11, last_mesh_graph.nodes[7].key);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(4.5, 4.5, 3.5)),
      RosToGtsam(last_mesh_graph.nodes[7].pose),
      1e-4));
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_mesh_frontend");
  return RUN_ALL_TESTS();
}