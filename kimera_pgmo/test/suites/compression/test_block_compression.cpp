/**
 * @file   test_block_compression.cpp
 * @brief  Unit-tests for mesh compression using voxel-block-based hashing
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include "kimera_pgmo/compression/block_compression.h"
#include "kimera_pgmo/utils/pcl_mesh_interface.h"
#include "pgmo_fixtures.h"

namespace kimera_pgmo {

TEST(TestBlockCompression, constructor) {
  BlockCompression compression(1.0);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr active_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  std::shared_ptr<std::vector<pcl::Vertices>> triangles(new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double>> vertex_timestamps(new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  EXPECT_EQ(0u, vertices->points.size());
  EXPECT_EQ(0u, active_vertices->points.size());
  EXPECT_EQ(0u, triangles->size());
  EXPECT_EQ(0u, vertex_timestamps->size());
}

TEST(TestBlockCompression, returnedValues) {
  BlockCompression compression(0.1);

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;
  HashedIndexMapping index_remappings;

  auto mesh = test::createSimpleMeshInterface(1.0);
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  // Check the partial integration
  EXPECT_EQ(5u, new_vertices.size());
  EXPECT_EQ(4u, new_triangles.size());
  EXPECT_EQ(5u, new_indices.size());
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
  EXPECT_EQ(4u, new_vertices.size());
  EXPECT_EQ(4u, new_triangles.size());
  EXPECT_EQ(5u, new_indices.size());
  expected_indices = {0, 5, 6, 7, 8};

  EXPECT_EQ(expected_indices, new_indices);
  EXPECT_EQ(2, new_vertices.points[0].x);
  EXPECT_EQ(2, new_vertices.points[2].y);
  EXPECT_EQ(2, new_vertices.points[3].z);
  EXPECT_EQ(0, new_triangles.at(0).vertices[0]);
  EXPECT_EQ(6, new_triangles.at(3).vertices[2]);
}

TEST(TestBlockCompression, storedValues) {
  BlockCompression compression(0.1);

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
  std::shared_ptr<std::vector<pcl::Vertices>> triangles(new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double>> vertex_timestamps(new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  // Check the stored interated values
  EXPECT_EQ(5u, vertices->size());
  EXPECT_EQ(5u, active_vertices->size());
  EXPECT_EQ(4u, triangles->size());
  EXPECT_EQ(5u, vertex_timestamps->size());

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

  EXPECT_EQ(9u, vertices->size());
  EXPECT_EQ(9u, active_vertices->size());
  EXPECT_EQ(8u, triangles->size());
  EXPECT_EQ(9u, vertex_timestamps->size());

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

TEST(TestBlockCompression, pruneStoredMesh) {
  BlockCompression compression(0.1);

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
  std::shared_ptr<std::vector<pcl::Vertices>> triangles(new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double>> vertex_timestamps(new std::vector<double>);

  // try pruning
  compression.pruneStoredMesh(100.5);
  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  EXPECT_EQ(5u, active_vertices->size());
  EXPECT_EQ(8u, triangles->size());
  EXPECT_EQ(5u, vertex_timestamps->size());
  EXPECT_EQ(9u, vertices->size());

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

  EXPECT_EQ(9u, active_vertices->size());
  EXPECT_EQ(12u, triangles->size());
  EXPECT_EQ(9u, vertex_timestamps->size());
  EXPECT_EQ(13u, vertices->size());

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
  EXPECT_EQ(4u, new_vertices.size());
  EXPECT_EQ(4u, new_triangles.size());
  EXPECT_EQ(5u, new_indices.size());
  std::vector<size_t> expected_indices = {0, 9, 10, 11, 12};

  EXPECT_EQ(expected_indices, new_indices);
  EXPECT_EQ(1, new_vertices.points[0].x);
  EXPECT_EQ(1, new_vertices.points[2].y);
  EXPECT_EQ(1, new_vertices.points[3].z);
  EXPECT_EQ(0, new_triangles.at(0).vertices[0]);
  EXPECT_EQ(11, new_triangles.at(1).vertices[1]);
  EXPECT_EQ(10, new_triangles.at(3).vertices[2]);
}

TEST(TestBlockCompression, returnedValuesCompressed) {
  BlockCompression compression(10.0);

  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;
  HashedIndexMapping index_remappings;

  auto mesh = test::createSimpleMeshInterface(1.0);
  compression.compressAndIntegrate(
      mesh, new_vertices, new_triangles, new_indices, index_remappings, 100.0);

  // Check the partial integration
  EXPECT_EQ(0u, new_vertices.size());
  EXPECT_EQ(0u, new_triangles.size());
  EXPECT_EQ(0u, new_indices.size());
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
  EXPECT_EQ(0u, new_vertices.size());
  EXPECT_EQ(0u, new_triangles.size());
  EXPECT_EQ(0u, new_indices.size());
  expected_indices = {};

  EXPECT_EQ(expected_indices, new_indices);
}

TEST(TestBlockCompression, storedValuesCompressed) {
  BlockCompression compression(10.0);

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
  std::shared_ptr<std::vector<pcl::Vertices>> triangles(new std::vector<pcl::Vertices>);
  std::shared_ptr<std::vector<double>> vertex_timestamps(new std::vector<double>);

  compression.getVertices(vertices);
  compression.getActiveVertices(active_vertices);
  compression.getStoredPolygons(triangles);
  compression.getActiveVerticesTimestamps(vertex_timestamps);

  // Check the stored interated values
  EXPECT_EQ(0u, vertices->size());
  EXPECT_EQ(0u, active_vertices->size());
  EXPECT_EQ(0u, triangles->size());
  EXPECT_EQ(0u, vertex_timestamps->size());

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

  EXPECT_EQ(0u, vertices->size());
  EXPECT_EQ(0u, active_vertices->size());
  EXPECT_EQ(0u, vertex_timestamps->size());
}

namespace {
class IndexedTestMesh : public MeshInterface {
 public:
  const BlockIndices& blockIndices() const override { return blocks_; }
  void markBlockActive(const BlockIndex& index) const override {
    offset_ = 10 * index.x();
  }
  size_t activeBlockSize() const override { return points_.size(); }
  pcl::PointXYZRGBA getActiveVertex(size_t i) const override {
    const auto& p = points_[i];
    pcl::PointXYZRGBA result;
    result.x = p[0] + offset_;
    result.y = p[1];
    result.z = p[2];
    return result;
  }
  size_t activeBlockNumFaces() const override { return faces_.size(); }
  std::array<size_t, 3> getActiveFace(size_t i) const override { return faces_.at(i); }
  Ptr clone() const override { return std::make_shared<IndexedTestMesh>(*this); }
  std::vector<std::array<size_t, 3>> faces_{{0, 2, 1}, {1, 2, 3}};

 private:
  mutable float offset_ = 0;
  BlockIndices blocks_{BlockIndex(0, 0, 0), BlockIndex(1, 0, 0)};
  std::vector<std::array<float, 3>> points_{
      {0, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 1, 0}, {100, 100, 100}};
};
}  // namespace

TEST(TestBlockCompression, IndexedFacesCorrectAcrossBlocksAndPruning) {
  BlockCompression compression(0.1);
  IndexedTestMesh mesh;
  const std::array<std::array<float, 3>, 8> expected_points{{{0, 0, 0},
                                                             {1, 0, 0},
                                                             {0, 1, 0},
                                                             {1, 1, 0},
                                                             {10, 0, 0},
                                                             {11, 0, 0},
                                                             {10, 1, 0},
                                                             {11, 1, 0}}};
  const std::array<std::array<size_t, 3>, 4> expected_faces{
      {{0, 2, 1}, {1, 2, 3}, {4, 6, 5}, {5, 6, 7}}};
  for (const auto stamp : {100.0, 101.0, 120.0}) {
    compression.pruneStoredMesh(stamp - 10);
    pcl::PointCloud<pcl::PointXYZRGBA> vertices;
    std::vector<pcl::Vertices> faces;
    std::vector<size_t> indices;
    HashedIndexMapping mapping;
    compression.compressAndIntegrate(mesh, vertices, faces, indices, mapping, stamp);

    const size_t offset = stamp == 120.0 ? 8 : 0;
    ASSERT_EQ(indices.size(), 8u);
    for (size_t i = 0; i < indices.size(); ++i) {
      EXPECT_EQ(indices[i], offset + i);
    }
    for (const auto& block : mesh.blockIndices()) {
      const auto& block_mapping = mapping.at(block);
      ASSERT_EQ(block_mapping.size(), 4u);
      EXPECT_EQ(block_mapping.count(4), 0u);  // Isolated input vertex.
      for (size_t i = 0; i < 4; ++i) {
        EXPECT_EQ(block_mapping.at(i), offset + 4 * block.x() + i);
      }
    }
    if (stamp == 101.0) {
      EXPECT_TRUE(vertices.empty());
      EXPECT_TRUE(faces.empty());
      continue;
    }

    ASSERT_EQ(vertices.size(), expected_points.size());
    for (size_t i = 0; i < expected_points.size(); ++i) {
      EXPECT_FLOAT_EQ(vertices[i].x, expected_points[i][0]);
      EXPECT_FLOAT_EQ(vertices[i].y, expected_points[i][1]);
      EXPECT_FLOAT_EQ(vertices[i].z, expected_points[i][2]);
    }
    ASSERT_EQ(faces.size(), expected_faces.size());
    for (size_t i = 0; i < expected_faces.size(); ++i) {
      ASSERT_EQ(faces[i].vertices.size(), 3u);
      for (size_t j = 0; j < 3; ++j) {
        EXPECT_EQ(faces[i].vertices[j], offset + expected_faces[i][j]);
      }
    }
  }
}

TEST(TestBlockCompression, RejectsIndexedFaceOutsideItsBlock) {
  BlockCompression compression(0.1);
  IndexedTestMesh mesh;
  mesh.faces_.push_back({0, 1, 5});
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  std::vector<pcl::Vertices> faces;
  std::vector<size_t> indices;
  HashedIndexMapping mapping;
  EXPECT_THROW(
      compression.compressAndIntegrate(mesh, vertices, faces, indices, mapping, 1.0),
      std::out_of_range);
}

}  // namespace kimera_pgmo
