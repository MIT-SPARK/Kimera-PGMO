/**
 * @file   testCommonFunctions.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include "gtest/gtest.h"

#include <mesh_msgs/TriangleMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

#include "mesher_mapper/CommonFunctions.h"
#include "test_config.h"

namespace mesher_mapper {
TEST(CommonFunctions, testReadPLY) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", mesh);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh->cloud, cloud);

  pcl::PointXYZ first_point(0, 0, -127);
  pcl::PointXYZ last_point(0, 0, 127);

  // Check number of surfaces
  EXPECT_EQ(size_t(840), mesh->polygons.size());
  EXPECT_EQ(size_t(422), cloud.points.size());

  // Check polygons parsed correctly
  EXPECT_EQ(0, mesh->polygons[0].vertices[0]);
  EXPECT_EQ(2, mesh->polygons[0].vertices[2]);
  EXPECT_EQ(420, mesh->polygons[839].vertices[0]);
  EXPECT_EQ(391, mesh->polygons[839].vertices[2]);

  // Check vertices parsed correctly
  EXPECT_EQ(-127, cloud.points[0].z);
  EXPECT_EQ(127, cloud.points[421].z);
}

TEST(CommonFunctions, PCLtoMeshMsg) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", mesh);

  // Convert to triangle mesh msg
  mesh_msgs::TriangleMesh triangle_mesh = PolygonMeshToTriangleMeshMsg(*mesh);

  // Convert back
  pcl::PolygonMesh new_mesh = TriangleMeshMsgToPolygonMesh(triangle_mesh);

  EXPECT_EQ(mesh->cloud.data, new_mesh.cloud.data);
  EXPECT_EQ(mesh->polygons[0].vertices[2], new_mesh.polygons[0].vertices[2]);
  EXPECT_EQ(mesh->polygons[100].vertices[1],
            new_mesh.polygons[100].vertices[1]);
  EXPECT_EQ(mesh->polygons[839].vertices[0],
            new_mesh.polygons[839].vertices[0]);
}
}  // namespace mesher_mapper