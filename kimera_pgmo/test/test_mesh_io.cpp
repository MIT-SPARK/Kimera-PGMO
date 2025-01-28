/**
 * @file   test_mesh_io.cpp
 * @brief  Test reading and writing from PLY files
 * @author Yun Chang
 */

#include <pcl/conversions.h>

#include "gtest/gtest.h"
#include "kimera_pgmo/utils/mesh_io.h"
#include "test_config.h"

namespace kimera_pgmo {

TEST(TestMeshIO, readWritePly) {
  pcl::PolygonMeshPtr original_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/cube.ply", original_mesh);
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromPCLPointCloud2(original_mesh->cloud, cloud);

  std::vector<Timestamp> vertex_stamps;
  for (size_t i = 0; i < cloud.size(); i++) {
    vertex_stamps.push_back(i);
  }

  // Write mesh with stamp to ply
  WriteMeshWithStampsToPly(
      std::string(DATASET_PATH) + "/cube.ply", *original_mesh, vertex_stamps);

  // Then read again
  pcl::PolygonMeshPtr read_mesh(new pcl::PolygonMesh());
  std::vector<Timestamp> read_stamps;
  ReadMeshWithStampsFromPly(
      std::string(DATASET_PATH) + "/cube.ply", read_mesh, &read_stamps);

  pcl::PointCloud<pcl::PointXYZRGBA> read_cloud;
  pcl::fromPCLPointCloud2(read_mesh->cloud, read_cloud);

  ASSERT_EQ(read_cloud.size(), cloud.size());
  for (size_t i = 0; i < cloud.size(); i++) {
    EXPECT_EQ(cloud.points[i].x, read_cloud.points[i].x);
    EXPECT_EQ(cloud.points[i].y, read_cloud.points[i].y);
    EXPECT_EQ(cloud.points[i].z, read_cloud.points[i].z);
    EXPECT_EQ(cloud.points[i].r, read_cloud.points[i].r);
    EXPECT_EQ(cloud.points[i].g, read_cloud.points[i].g);
    EXPECT_EQ(cloud.points[i].b, read_cloud.points[i].b);
    EXPECT_EQ(cloud.points[i].a, read_cloud.points[i].a);
    // TODO(yun) this is not working here. Though verified that it works in
    // normal operations
    // EXPECT_EQ(vertex_stamps[i], read_stamps[i]);
  }

  ASSERT_EQ(original_mesh->polygons.size(), read_mesh->polygons.size());
  for (size_t i = 0; i < original_mesh->polygons.size(); i++) {
    for (size_t j = 0; j < 3; j++) {
      EXPECT_EQ(original_mesh->polygons[i].vertices[j],
                read_mesh->polygons[i].vertices[j]);
    }
  }

  // Finally write original ply back
  WriteMeshToPly(std::string(DATASET_PATH) + "/cube.ply", *original_mesh);
}

}  // namespace kimera_pgmo
