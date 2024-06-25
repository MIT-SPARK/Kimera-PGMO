/**
 * @file   test_common_functions.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <ros/package.h>

#include <filesystem>

#include "kimera_pgmo_ros/conversion/mesh_conversion.h"

namespace kimera_pgmo {

std::filesystem::path getPgmoPath() {
  const auto path = ros::package::getPath("kimera_pgmo");
  return path;
}

TEST(TestMeshConversion, PCLtoMeshMsg) {
  const auto sphere_path = getPgmoPath() / "test" / "data" / "sphere.ply";
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(sphere_path, mesh);
  ASSERT_TRUE(mesh);

  // Convert to triangle mesh msg
  std::vector<Timestamp> orig_stamps;
  const auto msg = conversions::toMsg(0, *mesh, orig_stamps, "world");
  ASSERT_TRUE(msg);

  // Convert back
  std::vector<Timestamp> stamps;
  std::vector<int> vertex_graph_indices;
  const auto new_mesh = conversions::fromMsg(*msg, &stamps, &vertex_graph_indices);
  pcl::PointCloud<pcl::PointXYZRGBA> converted_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, converted_vertices);

  EXPECT_EQ(422, converted_vertices.points.size());
  EXPECT_EQ(-127, converted_vertices.points[0].z);
  EXPECT_EQ(5, converted_vertices.points[1].x);
  EXPECT_EQ(-1, converted_vertices.points[420].x);
  EXPECT_EQ(127, converted_vertices.points[421].z);
  EXPECT_EQ(mesh->polygons[0].vertices[2], new_mesh.polygons[0].vertices[2]);
  EXPECT_EQ(mesh->polygons[100].vertices[1], new_mesh.polygons[100].vertices[1]);
  EXPECT_EQ(mesh->polygons[839].vertices[0], new_mesh.polygons[839].vertices[0]);
}

}  // namespace kimera_pgmo
