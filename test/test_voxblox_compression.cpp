/**
 * @file   test_common_functions.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include "gtest/gtest.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

#include "kimera_pgmo/compression/VoxbloxCompression.h"

namespace kimera_pgmo {

namespace {

pcl::PolygonMesh createMesh(double scale) {
  // Create simple pcl mesh
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZRGBA> ptcld;
  pcl::PointXYZRGBA v0, v1, v2, v3, v4;
  v0.x = scale * 0;
  v0.y = scale * 0;
  v0.z = scale * 0;
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
  tri_1.vertices = std::vector<uint>{0, 1, 2};
  tri_2.vertices = std::vector<uint>{1, 3, 2};
  tri_3.vertices = std::vector<uint>{0, 1, 4};
  tri_4.vertices = std::vector<uint>{0, 4, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1, tri_2, tri_3, tri_4};

  return mesh;
}

} // namespace

TEST(test_voxblox_compression, constructor) {
  VoxbloxCompression compression(1.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices> > triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double> > vertex_timestamps(
      new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  EXPECT_EQ(size_t(0), vertices->points.size());
  EXPECT_EQ(size_t(0), active_vertices->points.size());
  EXPECT_EQ(size_t(0), triangles->size());
  EXPECT_EQ(size_t(0), vertex_timestamps->size());
}

TEST(test_voxblox_compression, returnedValues) {
  VoxbloxCompression compression(0.1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<size_t> > new_indices(new std::vector<size_t>);

  pcl::PolygonMesh mesh = createMesh(1.0);
  std::shared_ptr<std::unordered_map<size_t, size_t> > index_remappings =
      std::make_shared<std::unordered_map<size_t, size_t> >();
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  // Check the partial integration
  EXPECT_EQ(size_t(5), new_vertices->size());
  EXPECT_EQ(size_t(4), new_triangles->size());
  EXPECT_EQ(size_t(5), new_indices->size());
  std::vector<size_t> expected_indices = {0, 1, 2, 3, 4};

  EXPECT_EQ(expected_indices, *new_indices);
  EXPECT_EQ(1, new_vertices->points[1].x);
  EXPECT_EQ(1, new_vertices->points[2].y);
  EXPECT_EQ(1, new_vertices->points[4].z);
  EXPECT_EQ(0, new_triangles->at(0).vertices[0]);
  EXPECT_EQ(2, new_triangles->at(3).vertices[2]);

  // Insert another
  mesh = createMesh(2.0);
  new_vertices->clear();
  new_indices->clear();
  new_triangles->clear();
  index_remappings->clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  // Check the partial integration
  EXPECT_EQ(size_t(4), new_vertices->size());
  EXPECT_EQ(size_t(4), new_triangles->size());
  EXPECT_EQ(size_t(5), new_indices->size());
  expected_indices = {0, 5, 6, 7, 8};

  EXPECT_EQ(expected_indices, *new_indices);
  EXPECT_EQ(2, new_vertices->points[0].x);
  EXPECT_EQ(2, new_vertices->points[2].y);
  EXPECT_EQ(2, new_vertices->points[3].z);
  EXPECT_EQ(0, new_triangles->at(0).vertices[0]);
  EXPECT_EQ(6, new_triangles->at(3).vertices[2]);
}

TEST(test_voxblox_compression, storedValues) {
  VoxbloxCompression compression(0.1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<size_t> > new_indices(new std::vector<size_t>);

  pcl::PolygonMesh mesh = createMesh(1.0);
  std::shared_ptr<std::unordered_map<size_t, size_t> > index_remappings =
      std::make_shared<std::unordered_map<size_t, size_t> >();
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices> > triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double> > vertex_timestamps(
      new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  // Check the stored interated values
  EXPECT_EQ(size_t(5), vertices->size());
  EXPECT_EQ(size_t(5), active_vertices->size());
  EXPECT_EQ(size_t(4), triangles->size());
  EXPECT_EQ(size_t(5), vertex_timestamps->size());

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
  mesh = createMesh(2.0);
  new_vertices->clear();
  new_indices->clear();
  new_triangles->clear();
  index_remappings->clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  EXPECT_EQ(size_t(9), vertices->size());
  EXPECT_EQ(size_t(9), active_vertices->size());
  EXPECT_EQ(size_t(8), triangles->size());
  EXPECT_EQ(size_t(9), vertex_timestamps->size());

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

TEST(test_voxblox_compression, pruneStoredMesh) {
  VoxbloxCompression compression(0.1);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<size_t> > new_indices(new std::vector<size_t>);

  pcl::PolygonMesh mesh = createMesh(1.0);
  std::shared_ptr<std::unordered_map<size_t, size_t> > index_remappings =
      std::make_shared<std::unordered_map<size_t, size_t> >();
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  // Insert another
  mesh = createMesh(2.0);
  new_vertices->clear();
  new_indices->clear();
  new_triangles->clear();
  index_remappings->clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices> > triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double> > vertex_timestamps(
      new std::vector<double>);

  // try pruning
  compression.pruneStoredMesh(100.5);
  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  EXPECT_EQ(size_t(5), active_vertices->size());
  EXPECT_EQ(size_t(8), triangles->size());
  EXPECT_EQ(size_t(5), vertex_timestamps->size());
  EXPECT_EQ(size_t(9), vertices->size());

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
  mesh = createMesh(1.0);
  new_vertices->clear();
  new_indices->clear();
  new_triangles->clear();
  index_remappings->clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 102.0);
  compression.pruneStoredMesh(100.9);
  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  EXPECT_EQ(size_t(9), active_vertices->size());
  EXPECT_EQ(size_t(12), triangles->size());
  EXPECT_EQ(size_t(9), vertex_timestamps->size());
  EXPECT_EQ(size_t(13), vertices->size());

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
  EXPECT_EQ(size_t(4), new_vertices->size());
  EXPECT_EQ(size_t(4), new_triangles->size());
  EXPECT_EQ(size_t(5), new_indices->size());
  std::vector<size_t> expected_indices = {0, 9, 10, 11, 12};

  EXPECT_EQ(expected_indices, *new_indices);
  EXPECT_EQ(1, new_vertices->points[0].x);
  EXPECT_EQ(1, new_vertices->points[2].y);
  EXPECT_EQ(1, new_vertices->points[3].z);
  EXPECT_EQ(0, new_triangles->at(0).vertices[0]);
  EXPECT_EQ(11, new_triangles->at(1).vertices[1]);
  EXPECT_EQ(10, new_triangles->at(3).vertices[2]);
}

TEST(test_voxblox_compression, returnedValuesCompressed) {
  VoxbloxCompression compression(10.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<size_t> > new_indices(new std::vector<size_t>);

  pcl::PolygonMesh mesh = createMesh(1.0);
  std::shared_ptr<std::unordered_map<size_t, size_t> > index_remappings =
      std::make_shared<std::unordered_map<size_t, size_t> >();
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  // Check the partial integration
  EXPECT_EQ(size_t(0), new_vertices->size());
  EXPECT_EQ(size_t(0), new_triangles->size());
  EXPECT_EQ(size_t(0), new_indices->size());
  std::vector<size_t> expected_indices = {};

  EXPECT_EQ(expected_indices, *new_indices);

  // Insert another
  mesh = createMesh(2.0);
  new_vertices->clear();
  new_indices->clear();
  new_triangles->clear();
  index_remappings->clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  // Check the partial integration
  EXPECT_EQ(size_t(0), new_vertices->size());
  EXPECT_EQ(size_t(0), new_triangles->size());
  EXPECT_EQ(size_t(0), new_indices->size());
  expected_indices = {};

  EXPECT_EQ(expected_indices, *new_indices);
}

TEST(test_voxblox_compression, storedValuesCompressed) {
  VoxbloxCompression compression(10.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<size_t> > new_indices(new std::vector<size_t>);

  pcl::PolygonMesh mesh = createMesh(1.0);
  std::shared_ptr<std::unordered_map<size_t, size_t> > index_remappings =
      std::make_shared<std::unordered_map<size_t, size_t> >();
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices> > triangles(
      new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double> > vertex_timestamps(
      new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  // Check the stored interated values
  EXPECT_EQ(size_t(0), vertices->size());
  EXPECT_EQ(size_t(0), active_vertices->size());
  EXPECT_EQ(size_t(0), triangles->size());
  EXPECT_EQ(size_t(0), vertex_timestamps->size());

  // Insert another
  mesh = createMesh(2.0);
  new_vertices->clear();
  new_indices->clear();
  new_triangles->clear();
  index_remappings->clear();

  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 101.0);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  EXPECT_EQ(size_t(0), vertices->size());
  EXPECT_EQ(size_t(0), active_vertices->size());
  EXPECT_EQ(size_t(0), vertex_timestamps->size());
}

}  // namespace kimera_pgmo
