/**
 * @file   test_voxel_clearing_compression.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include <gtest/gtest.h>

#include "kimera_pgmo/compression/voxel_clearing_compression.h"
#include "pgmo_fixtures.h"

namespace kimera_pgmo {

using Cloud = pcl::PointCloud<pcl::PointXYZRGBA>;

test::BlockConfig block1_empty{"block1_empty", {0, 0, 0}, {}};
test::BlockConfig block1_test1{
    "block1_v1",
    {0, 0, 0},
    {{{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
     {{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
     {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

test::BlockConfig block2_empty{"block2_empty", {-1, 0, 0}, {}};
test::BlockConfig block2_test1{
    "block2_v1",
    {-1, 0, 0},
    {{{{-0.5, 0.5, 0.5}, {-0.5, 0.75, 0.75}, {-0.5, 0.75, 0.5}}},
     {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

struct CompressionInputs {
  MeshCompression::PointCloud vertices;
  std::vector<pcl::Vertices> triangles;
  std::vector<size_t> indices;
  HashedIndexMapping remappings;
};

struct CompressionOutput {
  explicit CompressionOutput(VoxelClearingCompression &compression)
      : vertices(new Cloud()),
        triangles(new std::vector<pcl::Vertices>()),
        timestamps(new std::vector<Timestamp>) {
    compression.getVertices(vertices);
    compression.getStoredPolygons(triangles);
    invalidated = compression.getInvalidIndices();
    active_indices = compression.getActiveVerticesIndex();
    compression.getTimestamps(timestamps);
  }

  Cloud::Ptr vertices;
  std::shared_ptr<std::vector<pcl::Vertices>> triangles;
  std::vector<size_t> invalidated;
  std::vector<size_t> active_indices;
  std::shared_ptr<std::vector<Timestamp>> timestamps;
};

bool checkTriangles(const std::vector<std::vector<uint32_t>> &expected,
                    const std::vector<pcl::Vertices> &result) {
  std::vector<std::vector<uint32_t>> result_filtered;
  for (size_t i = 0; i < result.size(); ++i) {
    if (result[i].vertices.size() != 3) {
      continue;
    }

    bool was_seen = false;
    for (const auto &triangle : result_filtered) {
      was_seen |= triangle[0] == result[i].vertices[0] &&
                  triangle[1] == result[i].vertices[1] &&
                  triangle[2] == result[i].vertices[2];
      if (was_seen) {
        break;
      }
    }

    if (was_seen) {
      continue;
    }

    std::vector<uint32_t> new_triangle = result[i].vertices;
    result_filtered.push_back(new_triangle);
  }

  EXPECT_EQ(expected.size(), result_filtered.size());
  if (expected.size() != result_filtered.size()) {
    throw std::logic_error("sizes don't match");
  }

  bool all_valid = true;
  for (size_t i = 0; i < expected.size(); ++i) {
    EXPECT_EQ(3u, expected[i].size()) << " @ " << i;
    EXPECT_EQ(3u, result_filtered[i].size()) << " @ " << i;

    std::set<uint32_t> expected_set(expected[i].begin(), expected[i].end());
    std::set<uint32_t> result_set(result_filtered[i].begin(), result_filtered[i].end());
    EXPECT_EQ(expected_set, result_set) << " @ " << i;
    all_valid &= expected_set == result_set;
  }

  return all_valid;
}

const double compression_factor = 1.0e-3;

TEST(TestVoxelClearingCompression, constructor) {
  VoxelClearingCompression compression(1.0);

  CompressionOutput output(compression);
  EXPECT_EQ(0u, output.vertices->points.size());
  EXPECT_EQ(0u, output.triangles->size());
  EXPECT_EQ(0u, output.invalidated.size());
  EXPECT_EQ(0u, output.active_indices.size());
  EXPECT_EQ(output.vertices->size(), output.timestamps->size());
}

TEST(TestVoxelClearingCompression, clearingCorrectBasic) {
  VoxelClearingCompression compression(compression_factor);

  auto mesh = test::createMesh({block1_test1});
  ASSERT_TRUE(mesh);

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    std::vector<size_t> expected_indices{0, 1, 2, 3, 4, 5};
    EXPECT_EQ(expected_indices, output.active_indices);

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  mesh = test::createMesh({block1_empty});
  ASSERT_TRUE(mesh);
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should clear everything
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(6u, output.invalidated.size());
    EXPECT_EQ(0u, output.triangles->size());
    EXPECT_EQ(0u, output.active_indices.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());
  }
}

TEST(TestVoxelClearingCompression, pruningCorrectBasic) {
  VoxelClearingCompression compression(compression_factor);

  auto mesh = test::createMesh({block1_test1});
  ASSERT_TRUE(mesh);
  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    std::vector<size_t> expected_indices{0, 1, 2, 3, 4, 5};
    EXPECT_EQ(expected_indices, output.active_indices);

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  compression.pruneStoredMesh(101.0);

  mesh = test::createMesh({block1_empty});
  ASSERT_TRUE(mesh);
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 105.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(0u, output.active_indices.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  mesh = test::createMesh({block1_test1});
  ASSERT_TRUE(mesh);
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 105.0);

    CompressionOutput output(compression);
    EXPECT_EQ(12u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    std::vector<size_t> expected_indices{6, 7, 8, 9, 10, 11};
    EXPECT_EQ(expected_indices, output.active_indices);
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}, {6, 7, 8}, {9, 10, 11}},
                               *output.triangles));
  }
}

TEST(TestVoxelClearingCompression, clearingCorrectMultiblock) {
  VoxelClearingCompression compression(compression_factor);

  auto mesh = test::createMesh({block1_test1, block2_test1});
  ASSERT_TRUE(mesh);

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(9u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{6, 7, 8}, {3, 4, 5}, {0, 1, 2}}, *output.triangles));
  }

  mesh = test::createMesh({block1_empty, block2_test1});
  ASSERT_TRUE(mesh);

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    // we should drop faces from the first block, but still keep faces that are
    // between both
    CompressionOutput output(compression);
    EXPECT_EQ(9u, output.vertices->points.size());
    EXPECT_EQ(3u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{6, 7, 8}, {3, 4, 5}}, *output.triangles));
  }

  mesh = test::createMesh({block1_test1, block2_test1});
  ASSERT_TRUE(mesh);

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    // we should be back at the original state
    CompressionOutput output(compression);
    EXPECT_EQ(12u, output.vertices->points.size());
    EXPECT_EQ(3u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{6, 7, 8}, {3, 4, 5}, {9, 10, 11}}, *output.triangles));
  }

  mesh = test::createMesh({block1_test1, block2_empty});
  ASSERT_TRUE(mesh);

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(12u, output.vertices->points.size());
    EXPECT_EQ(6u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{9, 10, 11}, {3, 4, 5}}, *output.triangles));
  }
}

TEST(TestVoxelClearingCompression, pruningCorrectMultiblock) {
  VoxelClearingCompression compression(compression_factor);

  auto mesh = test::createMesh({block1_test1});
  ASSERT_TRUE(mesh);

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  mesh = test::createMesh({block2_test1});
  ASSERT_TRUE(mesh);
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 102.0);

    CompressionOutput output(compression);
    EXPECT_EQ(9u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{6, 7, 8}, {3, 4, 5}, {0, 1, 2}}, *output.triangles));
  }

  compression.pruneStoredMesh(101.0);

  mesh = test::createMesh({block1_empty, block2_empty});
  ASSERT_TRUE(mesh);
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 105.0);

    CompressionOutput output(compression);
    EXPECT_EQ(9u, output.vertices->points.size());
    EXPECT_EQ(3u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  mesh = test::createMesh({block1_test1});
  ASSERT_TRUE(mesh);
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        *mesh, input.vertices, input.triangles, input.indices, input.remappings, 105.0);

    CompressionOutput output(compression);
    EXPECT_EQ(15u, output.vertices->points.size());
    EXPECT_EQ(3u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}, {9, 10, 11}, {12, 13, 14}},
                               *output.triangles));
  }
}

}  // namespace kimera_pgmo
