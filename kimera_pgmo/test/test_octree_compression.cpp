/**
 * @file   test_octree_compression.cpp
 * @brief  Unit-tests for oct-tree based compression
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include "kimera_pgmo/compression/octree_compression.h"
#include "kimera_pgmo/utils/pcl_mesh_interface.h"
#include "pgmo_fixtures.h"

namespace kimera_pgmo {

TEST(TestOctreeCompression, inOctreeBoundingBox) {
  pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> octree(1.0);
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices(new pcl::PointCloud<pcl::PointXYZ>());

  vertices->push_back(pcl::PointXYZ(0.0, 0.0, 0.0));
  vertices->push_back(pcl::PointXYZ(1.1, 1.1, 1.1));

  octree.setInputCloud(vertices);
  octree.addPointsFromInputCloud();

  EXPECT_TRUE(InOctreeBoundingBox<pcl::PointXYZ>(octree, pcl::PointXYZ(0.5, 0.5, 0.5)));
  EXPECT_FALSE(InOctreeBoundingBox<pcl::PointXYZ>(octree, pcl::PointXYZ(10, 10, 10)));
}

TEST(TestOctreeCompression, constructor) {
  OctreeCompression compression(1.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices> > triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double> > vertex_timestamps(new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  ASSERT_EQ(0u, vertices->points.size());
  ASSERT_EQ(0u, active_vertices->points.size());
  ASSERT_EQ(0u, triangles->size());
  ASSERT_EQ(0u, vertex_timestamps->size());
}

TEST(TestOctreeCompression, returnedValues) {
  OctreeCompression compression(0.1);

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;
  HashedIndexMapping index_remappings;

  auto mesh = test::createSimpleMeshInterface(1.0);
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  // Check the partial integration
  ASSERT_EQ(5u, new_vertices.size());
  ASSERT_EQ(4u, new_triangles.size());
  ASSERT_EQ(5u, new_indices.size());
  std::vector<size_t> expected_indices = {0, 1, 2, 3, 4};

  EXPECT_EQ(expected_indices, new_indices);
  EXPECT_EQ(1, new_vertices.points[1].x);
  EXPECT_EQ(1, new_vertices.points[2].y);
  EXPECT_EQ(1, new_vertices.points[4].z);
  EXPECT_EQ(0, new_triangles.at(0).vertices[0]);
  EXPECT_EQ(2, new_triangles.at(3).vertices[2]);

  // Insert another
  mesh = test::createSimpleMeshInterface(2.0);
  new_vertices.clear();
  new_indices.clear();
  new_triangles.clear();
  index_remappings.clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  // Check the partial integration
  ASSERT_EQ(4u, new_vertices.size());
  ASSERT_EQ(4u, new_triangles.size());
  ASSERT_EQ(5u, new_indices.size());
  expected_indices = {0, 5, 6, 7, 8};

  EXPECT_EQ(expected_indices, new_indices);
  EXPECT_EQ(2, new_vertices.points[0].x);
  EXPECT_EQ(2, new_vertices.points[2].y);
  EXPECT_EQ(2, new_vertices.points[3].z);
  EXPECT_EQ(0, new_triangles.at(0).vertices[0]);
  EXPECT_EQ(6, new_triangles.at(3).vertices[2]);
}

TEST(TestOctreeCompression, storedValues) {
  OctreeCompression compression(0.1);

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;
  HashedIndexMapping index_remappings;

  auto mesh = test::createSimpleMeshInterface(1.0);
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices> > triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double> > vertex_timestamps(new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  // Check the stored interated values
  ASSERT_EQ(5u, vertices->size());
  ASSERT_EQ(5u, active_vertices->size());
  ASSERT_EQ(4u, triangles->size());
  ASSERT_EQ(5u, vertex_timestamps->size());

  EXPECT_EQ(1, vertices->points[1].x);
  EXPECT_EQ(1, vertices->points[2].y);
  EXPECT_EQ(1, vertices->points[4].z);
  EXPECT_EQ(1, active_vertices->points[1].x);
  EXPECT_EQ(1, active_vertices->points[2].y);
  EXPECT_EQ(1, active_vertices->points[4].z);
  EXPECT_EQ(0, triangles->at(0).vertices[0]);
  EXPECT_EQ(2, triangles->at(3).vertices[2]);
  EXPECT_EQ(100.0, vertex_timestamps->at(4));

  // Insert another
  mesh = test::createSimpleMeshInterface(2.0);
  new_vertices.clear();
  new_indices.clear();
  new_triangles.clear();
  index_remappings.clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  ASSERT_EQ(9u, vertices->size());
  ASSERT_EQ(9u, active_vertices->size());
  ASSERT_EQ(8u, triangles->size());
  ASSERT_EQ(9u, vertex_timestamps->size());

  EXPECT_EQ(2, vertices->points[5].x);
  EXPECT_EQ(2, vertices->points[7].y);
  EXPECT_EQ(2, vertices->points[8].z);
  EXPECT_EQ(2, active_vertices->points[5].x);
  EXPECT_EQ(2, active_vertices->points[7].y);
  EXPECT_EQ(2, active_vertices->points[8].z);
  EXPECT_EQ(0, triangles->at(4).vertices[0]);
  EXPECT_EQ(6, triangles->at(7).vertices[2]);
  EXPECT_EQ(101.0, vertex_timestamps->at(8));
}

TEST(TestOctreeCompression, pruneStoredMesh) {
  OctreeCompression compression(0.1);

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;
  HashedIndexMapping index_remappings;

  auto mesh = test::createSimpleMeshInterface(1.0);
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  // Insert another
  mesh = test::createSimpleMeshInterface(2.0);
  new_vertices.clear();
  new_indices.clear();
  new_triangles.clear();
  index_remappings.clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices> > triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double> > vertex_timestamps(new std::vector<double>);

  // try pruning
  compression.pruneStoredMesh(100.5);
  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  ASSERT_EQ(5u, active_vertices->size());
  ASSERT_EQ(8u, triangles->size());
  ASSERT_EQ(5u, vertex_timestamps->size());
  ASSERT_EQ(9u, vertices->size());

  EXPECT_EQ(2, active_vertices->points[1].x);
  EXPECT_EQ(2, active_vertices->points[2].y);
  EXPECT_EQ(2, active_vertices->points[4].z);
  EXPECT_EQ(0, triangles->at(0).vertices[0]);
  EXPECT_EQ(2, triangles->at(3).vertices[2]);
  EXPECT_EQ(101.0, vertex_timestamps->at(4));
  EXPECT_EQ(101.0, vertex_timestamps->at(0));

  EXPECT_EQ(2, vertices->points[5].x);
  EXPECT_EQ(2, vertices->points[7].y);
  EXPECT_EQ(2, vertices->points[8].z);
  EXPECT_EQ(0, triangles->at(4).vertices[0]);
  EXPECT_EQ(6, triangles->at(7).vertices[2]);

  // Try insert after pruning
  mesh = test::createSimpleMeshInterface(1.0);
  new_vertices.clear();
  new_indices.clear();
  new_triangles.clear();
  index_remappings.clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 102.0);
  compression.pruneStoredMesh(100.9);
  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  ASSERT_EQ(9u, active_vertices->size());
  ASSERT_EQ(12u, triangles->size());
  ASSERT_EQ(9u, vertex_timestamps->size());
  ASSERT_EQ(13u, vertices->size());

  EXPECT_EQ(1, active_vertices->points[5].x);
  EXPECT_EQ(1, active_vertices->points[7].y);
  EXPECT_EQ(1, active_vertices->points[8].z);
  EXPECT_EQ(0, triangles->at(4).vertices[0]);
  EXPECT_EQ(6, triangles->at(7).vertices[2]);
  EXPECT_EQ(0, triangles->at(8).vertices[0]);
  EXPECT_EQ(10, triangles->at(11).vertices[2]);
  EXPECT_EQ(102.0, vertex_timestamps->at(8));

  // Test also the returned values
  // Check the partial integration
  ASSERT_EQ(4u, new_vertices.size());
  ASSERT_EQ(4u, new_triangles.size());
  ASSERT_EQ(5u, new_indices.size());
  std::vector<size_t> expected_indices = {0, 9, 10, 11, 12};

  EXPECT_EQ(expected_indices, new_indices);
  EXPECT_EQ(1, new_vertices.points[0].x);
  EXPECT_EQ(1, new_vertices.points[2].y);
  EXPECT_EQ(1, new_vertices.points[3].z);
  EXPECT_EQ(0, new_triangles.at(0).vertices[0]);
  EXPECT_EQ(11, new_triangles.at(1).vertices[1]);
  EXPECT_EQ(10, new_triangles.at(3).vertices[2]);
}

TEST(TestOctreeCompression, returnedValuesCompressed) {
  OctreeCompression compression(10.0);

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;
  HashedIndexMapping index_remappings;

  auto mesh = test::createSimpleMeshInterface(1.0);
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  // Check the partial integration
  ASSERT_EQ(0u, new_vertices.size());
  ASSERT_EQ(0u, new_triangles.size());
  ASSERT_EQ(0u, new_indices.size());
  std::vector<size_t> expected_indices = {};

  EXPECT_EQ(expected_indices, new_indices);

  // Insert another
  mesh = test::createSimpleMeshInterface(2.0);
  new_vertices.clear();
  new_indices.clear();
  new_triangles.clear();
  index_remappings.clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  // Check the partial integration
  ASSERT_EQ(0u, new_vertices.size());
  ASSERT_EQ(0u, new_triangles.size());
  ASSERT_EQ(0u, new_indices.size());
  expected_indices = {};

  EXPECT_EQ(expected_indices, new_indices);
}

TEST(TestOctreeCompression, storedValuesCompressed) {
  OctreeCompression compression(10.0);

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;
  HashedIndexMapping index_remappings;

  auto mesh = test::createSimpleMeshInterface(1.0);
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices> > triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double> > vertex_timestamps(new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  // Check the stored interated values
  ASSERT_EQ(0u, vertices->size());
  ASSERT_EQ(0u, active_vertices->size());
  ASSERT_EQ(0u, triangles->size());
  ASSERT_EQ(0u, vertex_timestamps->size());

  // Insert another
  mesh = test::createSimpleMeshInterface(2.0);
  new_vertices.clear();
  new_indices.clear();
  new_triangles.clear();
  index_remappings.clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  ASSERT_EQ(0u, vertices->size());
  ASSERT_EQ(0u, active_vertices->size());
  ASSERT_EQ(0u, vertex_timestamps->size());
}

}  // namespace kimera_pgmo
