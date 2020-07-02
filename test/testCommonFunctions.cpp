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

#include "kimera_pgmo/CommonFunctions.h"
#include "test_config.h"

namespace kimera_pgmo {
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
  pcl::PointCloud<pcl::PointXYZRGBA> converted_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, converted_vertices);

  EXPECT_EQ(422, converted_vertices.points.size());
  EXPECT_EQ(-127, converted_vertices.points[0].z);
  EXPECT_EQ(5, converted_vertices.points[1].x);
  EXPECT_EQ(-1, converted_vertices.points[420].x);
  EXPECT_EQ(127, converted_vertices.points[421].z);
  EXPECT_EQ(mesh->polygons[0].vertices[2], new_mesh.polygons[0].vertices[2]);
  EXPECT_EQ(mesh->polygons[100].vertices[1],
            new_mesh.polygons[100].vertices[1]);
  EXPECT_EQ(mesh->polygons[839].vertices[0],
            new_mesh.polygons[839].vertices[0]);
}

TEST(CommonFunctions, PolygonsEqual) {
  pcl::Vertices p0, p1, p2, p3;

  p0.vertices.push_back(0);
  p0.vertices.push_back(1);
  p0.vertices.push_back(2);

  p1.vertices.push_back(2);
  p1.vertices.push_back(0);
  p1.vertices.push_back(1);

  p2.vertices.push_back(0);
  p2.vertices.push_back(1);
  p2.vertices.push_back(3);

  p3.vertices.push_back(0);
  p3.vertices.push_back(1);
  p3.vertices.push_back(2);
  p3.vertices.push_back(4);

  EXPECT_TRUE(PolygonsEqual(p0, p0));
  EXPECT_TRUE(PolygonsEqual(p0, p1));
  EXPECT_FALSE(PolygonsEqual(p0, p2));
  EXPECT_FALSE(PolygonsEqual(p0, p3));
}
}  // namespace kimera_pgmo