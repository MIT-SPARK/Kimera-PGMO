/**
 * @file   test_voxblox_processing.cpp
 * @brief  Unit-tests for the graph structure
 * @author Yun Chang
 */
#include <ros/ros.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/MeshBlock.h>

#include "gtest/gtest.h"

#include "kimera_pgmo/VoxbloxProcessing.h"
#include "kimera_pgmo/utils/VoxbloxUtils.h"

namespace kimera_pgmo {

class VoxbloxProcessingTest : public ::testing::Test {
 protected:
  VoxbloxProcessingTest() { system("rosparam set horizon 1.0"); }

  ~VoxbloxProcessingTest() {}

  // Measurements callback
  void PruneStoredBlocks(const ros::Time& time) { vp_.pruneStoredBlocks(time); }

  // Test update called in timer event
  pcl::PolygonMesh ProcessVoxbloxMesh(const voxblox_msgs::Mesh::ConstPtr& msg) {
    return vp_.processVoxbloxMesh(msg);
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

  void AddMeshBlockDummy(const BlockIndex& idx,
                         const std::vector<size_t>& block_indices,
                         const ros::Time& stamp) {
    vp_.mesh_block_vertices_[idx] = block_indices;
    vp_.mesh_block_last_detection_[idx] = stamp;
  }

  void GetStoredMeshblocks(
      std::map<BlockIndex, ros::Time>* block_stamp,
      std::map<BlockIndex, std::vector<size_t>>* mesh_vertices) {
    *mesh_vertices = vp_.mesh_block_vertices_;
    *block_stamp = vp_.mesh_block_last_detection_;
  }

  void GetFullMesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices,
                   std::vector<pcl::Vertices>* triangles) {
    *vertices = *(vp_.vertices_);
    *triangles = vp_.triangles_;
  }

  VoxbloxProcessing vp_;
};

TEST_F(VoxbloxProcessingTest, initialize) {
  // Test initialization
  ros::NodeHandle nh;
  bool init = vp_.initialize(nh);
  ASSERT_TRUE(init);
}

TEST_F(VoxbloxProcessingTest, pruneStoredBlocks) {
  // Test pruning stored blocks
  ros::NodeHandle nh;
  vp_.initialize(nh);

  std::vector<size_t> block_indices_1 = {0, 1, 2, 3};
  std::vector<size_t> block_indices_2 = {3, 4, 6, 7};
  std::vector<size_t> block_indices_3 = {5, 9};

  AddMeshBlockDummy(BlockIndex(1, 1, 1), block_indices_1, ros::Time(0.39));
  PruneStoredBlocks(ros::Time(0.5));

  std::map<BlockIndex, ros::Time> block_stamps;
  std::map<BlockIndex, std::vector<size_t>> block_vertices;
  GetStoredMeshblocks(&block_stamps, &block_vertices);

  EXPECT_EQ(size_t(1), block_vertices.size());
  EXPECT_EQ(size_t(1), block_stamps.size());
  EXPECT_TRUE(block_vertices.find(BlockIndex(1, 1, 1)) != block_vertices.end());
  EXPECT_EQ(ros::Time(0.39), block_stamps[BlockIndex(1, 1, 1)]);
  EXPECT_EQ(block_indices_1, block_vertices[BlockIndex(1, 1, 1)]);

  AddMeshBlockDummy(BlockIndex(2, 14, 2), block_indices_2, ros::Time(1.0));
  PruneStoredBlocks(ros::Time(1.1));

  GetStoredMeshblocks(&block_stamps, &block_vertices);

  EXPECT_EQ(size_t(2), block_vertices.size());
  EXPECT_EQ(size_t(2), block_stamps.size());
  EXPECT_TRUE(block_vertices.find(BlockIndex(2, 14, 2)) !=
              block_vertices.end());
  EXPECT_EQ(ros::Time(1.0), block_stamps[BlockIndex(2, 14, 2)]);
  EXPECT_EQ(block_indices_2, block_vertices[BlockIndex(2, 14, 2)]);

  AddMeshBlockDummy(BlockIndex(3, 4, 3), block_indices_3, ros::Time(1.4));
  PruneStoredBlocks(ros::Time(1.4));

  GetStoredMeshblocks(&block_stamps, &block_vertices);

  EXPECT_EQ(size_t(2), block_vertices.size());
  EXPECT_EQ(size_t(2), block_stamps.size());
  EXPECT_TRUE(block_vertices.find(BlockIndex(3, 4, 3)) != block_vertices.end());
  EXPECT_TRUE(block_vertices.find(BlockIndex(1, 1, 1)) == block_vertices.end());
  EXPECT_TRUE(block_stamps.find(BlockIndex(1, 1, 1)) == block_stamps.end());
  EXPECT_EQ(ros::Time(1.4), block_stamps[BlockIndex(3, 4, 3)]);
  EXPECT_EQ(block_indices_3, block_vertices[BlockIndex(3, 4, 3)]);
}

TEST_F(VoxbloxProcessingTest, partialMesh) {
  // Test the construction of the martial mesh during callback
  ros::NodeHandle nh;
  vp_.initialize(nh);

  voxblox_msgs::Mesh::Ptr mesh1(new voxblox_msgs::Mesh);
  *mesh1 = CreateSimpleMesh1();

  pcl::PolygonMesh partial = ProcessVoxbloxMesh(mesh1);
  pcl::PointCloud<pcl::PointXYZRGBA> partial_vertices;
  pcl::fromPCLPointCloud2(partial.cloud, partial_vertices);

  EXPECT_EQ(size_t(4), partial_vertices.points.size());
  EXPECT_EQ(size_t(2), partial.polygons.size());
  // Test points and triangles
  EXPECT_NEAR(1, partial_vertices.points[1].x, 1e-3);
  EXPECT_NEAR(1, partial_vertices.points[3].x, 1e-3);
  EXPECT_NEAR(1, partial_vertices.points[3].y, 1e-3);
  EXPECT_EQ(2, partial.polygons[0].vertices[2]);
  EXPECT_EQ(1, partial.polygons[1].vertices[0]);

  // Add second mesh
  voxblox_msgs::Mesh::Ptr mesh2(new voxblox_msgs::Mesh);
  *mesh2 = CreateSimpleMesh2();
  partial = ProcessVoxbloxMesh(mesh2);
  pcl::fromPCLPointCloud2(partial.cloud, partial_vertices);

  EXPECT_EQ(size_t(12), partial_vertices.points.size());
  EXPECT_EQ(size_t(6), partial.polygons.size());
  // Test points and triangles
  EXPECT_NEAR(3, partial_vertices.points[5].x, 1e-3);
  EXPECT_NEAR(3, partial_vertices.points[7].x, 1e-3);
  EXPECT_NEAR(1, partial_vertices.points[7].y, 1e-3);
  EXPECT_EQ(6, partial.polygons[2].vertices[2]);
  EXPECT_EQ(5, partial.polygons[3].vertices[0]);

  EXPECT_NEAR(2, partial_vertices.points[9].y, 1e-3);
  EXPECT_NEAR(3, partial_vertices.points[11].x, 1e-3);
  EXPECT_NEAR(3, partial_vertices.points[11].y, 1e-3);
  EXPECT_EQ(10, partial.polygons[4].vertices[2]);
  EXPECT_EQ(9, partial.polygons[5].vertices[0]);

  // Add third mesh
  voxblox_msgs::Mesh::Ptr mesh3(new voxblox_msgs::Mesh);
  *mesh3 = CreateSimpleMesh3();
  partial = ProcessVoxbloxMesh(mesh3);
  pcl::fromPCLPointCloud2(partial.cloud, partial_vertices);

  EXPECT_EQ(size_t(8), partial_vertices.points.size());
  EXPECT_EQ(size_t(4), partial.polygons.size());
  // Test points and triangles
  EXPECT_NEAR(1, partial_vertices.points[1].x, 1e-3);
  EXPECT_NEAR(1, partial_vertices.points[3].x, 1e-3);
  EXPECT_NEAR(1, partial_vertices.points[3].y, 1e-3);
  EXPECT_EQ(2, partial.polygons[0].vertices[2]);
  EXPECT_EQ(1, partial.polygons[1].vertices[0]);

  EXPECT_NEAR(3, partial_vertices.points[5].x, 1e-3);
  EXPECT_NEAR(2, partial_vertices.points[7].z, 1e-3);
  EXPECT_NEAR(1, partial_vertices.points[7].y, 1e-3);
  EXPECT_EQ(6, partial.polygons[2].vertices[2]);
  EXPECT_EQ(5, partial.polygons[3].vertices[0]);
}

TEST_F(VoxbloxProcessingTest, fullMesh) {
  // Test the updated full mesh after callback
  ros::NodeHandle nh;
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
  // Check stored mesh block info
  std::map<BlockIndex, ros::Time> block_stamps;
  std::map<BlockIndex, std::vector<size_t>> block_vertices;
  GetStoredMeshblocks(&block_stamps, &block_vertices);
  std::vector<size_t> expected_block_indices_1 = {0, 1, 2, 3};

  EXPECT_EQ(size_t(1), block_vertices.size());
  EXPECT_EQ(size_t(1), block_stamps.size());
  EXPECT_TRUE(block_vertices.find(BlockIndex(0, 0, 0)) != block_vertices.end());
  EXPECT_EQ(ros::Time(10.0), block_stamps[BlockIndex(0, 0, 0)]);
  EXPECT_EQ(expected_block_indices_1, block_vertices[BlockIndex(0, 0, 0)]);

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

  GetStoredMeshblocks(&block_stamps, &block_vertices);
  std::vector<size_t> expected_block_indices_2 = {4, 5, 6, 7};
  std::vector<size_t> expected_block_indices_3 = {8, 9, 10, 11};

  EXPECT_EQ(size_t(3), block_vertices.size());
  EXPECT_EQ(size_t(3), block_stamps.size());
  EXPECT_EQ(ros::Time(10.5), block_stamps[BlockIndex(0, 0, 0)]);
  EXPECT_EQ(ros::Time(10.5), block_stamps[BlockIndex(1, 0, 0)]);
  EXPECT_EQ(ros::Time(10.5), block_stamps[BlockIndex(1, 1, 0)]);
  EXPECT_EQ(expected_block_indices_1, block_vertices[BlockIndex(0, 0, 0)]);
  EXPECT_EQ(expected_block_indices_2, block_vertices[BlockIndex(1, 0, 0)]);
  EXPECT_EQ(expected_block_indices_3, block_vertices[BlockIndex(1, 1, 0)]);

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

  GetStoredMeshblocks(&block_stamps, &block_vertices);
  std::vector<size_t> expected_block_indices_4 = {12, 13, 14, 15};
  std::vector<size_t> expected_block_indices_5 = {16, 17, 18, 19};

  EXPECT_EQ(size_t(2), block_vertices.size());
  EXPECT_EQ(size_t(2), block_stamps.size());
  EXPECT_EQ(ros::Time(11.6), block_stamps[BlockIndex(0, 0, 0)]);
  EXPECT_EQ(ros::Time(11.6), block_stamps[BlockIndex(1, 0, 1)]);
  EXPECT_EQ(expected_block_indices_4, block_vertices[BlockIndex(0, 0, 0)]);
  EXPECT_EQ(expected_block_indices_5, block_vertices[BlockIndex(1, 0, 1)]);
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_voxblox_processing");
  return RUN_ALL_TESTS();
}
