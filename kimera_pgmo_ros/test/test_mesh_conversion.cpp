/**
 * @file   test_common_functions.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <filesystem>

#include "kimera_pgmo_ros/conversion/mesh.h"

namespace kimera_pgmo {
namespace {

void fillMesh(pcl::PolygonMesh& mesh, double scale = 1.0) {
  pcl::PointCloud<pcl::PointXYZRGBA> ptcld;
  pcl::PointXYZRGBA v0, v1, v2, v3, v4;
  v0.x = scale * 1;
  v0.y = scale * 1;
  v0.z = scale * 1;
  v0.r = 23;
  v0.g = 24;
  v0.b = 122;
  v0.a = 255;

  v1.x = scale * 1;
  v1.y = scale * 0;
  v1.z = scale * 0;
  v1.r = 33;
  v1.g = 34;
  v1.b = 52;
  v1.a = 255;

  v2.x = scale * 0;
  v2.y = scale * 1;
  v2.z = scale * 0;
  v2.r = 12;
  v2.g = 144;
  v2.b = 22;
  v2.a = 255;

  v3.x = scale * 1;
  v3.y = scale * 1;
  v3.z = scale * 0;
  v3.r = 0;
  v3.g = 14;
  v3.b = 0;
  v3.a = 255;

  v4.x = scale * 0;
  v4.y = scale * 0;
  v4.z = scale * 1;
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
  tri_1.vertices = {0, 1, 2};
  tri_2.vertices = {1, 3, 2};
  tri_3.vertices = {0, 1, 4};
  tri_4.vertices = {0, 4, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1, tri_2, tri_3, tri_4};
}

}  // namespace

TEST(TestMeshConversion, PCLtoMeshMsg) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  fillMesh(*mesh);

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

  EXPECT_EQ(5u, converted_vertices.points.size());
  EXPECT_EQ(1.0, converted_vertices.points[0].z);
  EXPECT_EQ(1.0, converted_vertices.points[1].x);
  EXPECT_EQ(mesh->polygons[0].vertices[2], new_mesh.polygons[0].vertices[2]);
  EXPECT_EQ(mesh->polygons[1].vertices[1], new_mesh.polygons[1].vertices[1]);
  EXPECT_EQ(mesh->polygons[2].vertices[0], new_mesh.polygons[2].vertices[0]);
}

}  // namespace kimera_pgmo
