/**
 * @file   test_voxel_clearing_compression.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include "gtest/gtest.h"
#include "kimera_pgmo/compression/VoxelClearingCompression.h"

namespace kimera_pgmo {

using FaceCoordinates = std::array<std::array<float, 3>, 3>;
using Cloud = pcl::PointCloud<pcl::PointXYZRGBA>;

namespace {

inline void addPointToBlock(voxblox_msgs::MeshBlock &block,
                            const std::array<float, 3> &point) {
  const float a = std::numeric_limits<uint16_t>::max() / 2.0f;
  // x = (a * m_x + b_x) * l
  // m_x = 1 / a * (m_x / l - b_x)
  // for simplicity, l is 1
  block.x.push_back(static_cast<uint16_t>(a * (point[0] - block.index[0])));
  block.y.push_back(static_cast<uint16_t>(a * (point[1] - block.index[1])));
  block.z.push_back(static_cast<uint16_t>(a * (point[2] - block.index[2])));
  block.r.push_back(255);
  block.g.push_back(255);
  block.b.push_back(255);
}

struct BlockConfig {
  std::array<int64_t, 3> index;
  std::vector<FaceCoordinates> faces;

  voxblox_msgs::MeshBlock instantiate() const {
    voxblox_msgs::MeshBlock to_return;
    std::memcpy(to_return.index.data(), index.data(), sizeof(to_return.index));

    for (const auto &face : faces) {
      addPointToBlock(to_return, face[0]);
      addPointToBlock(to_return, face[1]);
      addPointToBlock(to_return, face[2]);
    }

    return to_return;
  }
};

BlockConfig block1_empty{{0, 0, 0}, {}};
BlockConfig block1_test1{{0, 0, 0},
                         {{{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
                          {{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
                          {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

BlockConfig block2_empty{{-1, 0, 0}, {}};
BlockConfig block2_test1{{-1, 0, 0},
                         {{{{-0.5, 0.5, 0.5}, {-0.5, 0.75, 0.75}, {-0.5, 0.75, 0.5}}},
                          {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

voxblox_msgs::Mesh createMesh(const std::vector<BlockConfig> &configs) {
  voxblox_msgs::Mesh mesh;
  mesh.block_edge_length = 1.0;
  for (const auto &config : configs) {
    mesh.mesh_blocks.push_back(config.instantiate());
  }
  return mesh;
}

struct CompressionInputs {
  CompressionInputs()
      : vertices(new Cloud()),
        triangles(new std::vector<pcl::Vertices>()),
        indices(new std::vector<size_t>()),
        remappings(new VoxbloxIndexMapping()) {}

  Cloud::Ptr vertices;
  std::shared_ptr<std::vector<pcl::Vertices>> triangles;
  std::shared_ptr<std::vector<size_t>> indices;
  std::shared_ptr<VoxbloxIndexMapping> remappings;
};

struct CompressionOutput {
  explicit CompressionOutput(VoxelClearingCompression &compression)
      : vertices(new Cloud()),
        triangles(new std::vector<pcl::Vertices>()),
        timestamps(new std::vector<ros::Time>) {
    compression.getVertices(vertices);
    compression.getStoredPolygons(triangles);
    invalidated = compression.getInvalidIndices();
    active_indices = compression.getActiveVerticesIndex();
    compression.getVertexStamps(timestamps);
  }

  Cloud::Ptr vertices;
  std::shared_ptr<std::vector<pcl::Vertices>> triangles;
  std::vector<size_t> invalidated;
  std::vector<size_t> active_indices;
  std::shared_ptr<std::vector<ros::Time>> timestamps;
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

}

const double compression_factor = 1.0e-3;

TEST(test_voxel_clearing_compression, constructor) {
  VoxelClearingCompression compression(1.0);

  CompressionOutput output(compression);
  EXPECT_EQ(0u, output.vertices->points.size());
  EXPECT_EQ(0u, output.triangles->size());
  EXPECT_EQ(0u, output.invalidated.size());
  EXPECT_EQ(0u, output.active_indices.size());
  EXPECT_EQ(output.vertices->size(), output.timestamps->size());
}

TEST(test_voxel_clearing_compression, clearingCorrectBasic) {
  VoxelClearingCompression compression(compression_factor);

  auto mesh = createMesh({block1_test1});

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    std::vector<size_t> expected_indices{0, 1, 2, 3, 4, 5};
    EXPECT_EQ(expected_indices, output.active_indices);

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  mesh = createMesh({block1_empty});
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should clear everything
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(6u, output.invalidated.size());
    EXPECT_EQ(0u, output.triangles->size());
    EXPECT_EQ(0u, output.active_indices.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());
  }
}

TEST(test_voxel_clearing_compression, pruningCorrectBasic) {
  VoxelClearingCompression compression(compression_factor);

  auto mesh = createMesh({block1_test1});

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    std::vector<size_t> expected_indices{0, 1, 2, 3, 4, 5};
    EXPECT_EQ(expected_indices, output.active_indices);

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  compression.pruneStoredMesh(101.0);

  mesh = createMesh({block1_empty});
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 105.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(0u, output.active_indices.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  mesh = createMesh({block1_test1});
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 105.0);

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

TEST(test_voxel_clearing_compression, clearingCorrectMultiblock) {
  VoxelClearingCompression compression(compression_factor);

  auto mesh = createMesh({block1_test1, block2_test1});

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(9u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{6, 7, 8}, {3, 4, 5}, {0, 1, 2}}, *output.triangles));
  }

  mesh = createMesh({block1_empty, block2_test1});

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    // we should drop faces from the first block, but still keep faces that are
    // between both
    CompressionOutput output(compression);
    EXPECT_EQ(9u, output.vertices->points.size());
    EXPECT_EQ(3u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{6, 7, 8}, {3, 4, 5}}, *output.triangles));
  }

  mesh = createMesh({block1_test1, block2_test1});

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    // we should be back at the original state
    CompressionOutput output(compression);
    EXPECT_EQ(12u, output.vertices->points.size());
    EXPECT_EQ(3u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{6, 7, 8}, {3, 4, 5}, {9, 10, 11}}, *output.triangles));
  }

  mesh = createMesh({block1_test1, block2_empty});

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(12u, output.vertices->points.size());
    EXPECT_EQ(6u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{9, 10, 11}, {3, 4, 5}}, *output.triangles));
  }
}

TEST(test_voxel_clearing_compression, pruningCorrectMultiblock) {
  VoxelClearingCompression compression(compression_factor);

  auto mesh = createMesh({block1_test1});

  {  // limit temporary scopes
    CompressionInputs input;
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 100.0);

    CompressionOutput output(compression);
    EXPECT_EQ(6u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  mesh = createMesh({block2_test1});
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 102.0);

    CompressionOutput output(compression);
    EXPECT_EQ(9u, output.vertices->points.size());
    EXPECT_EQ(0u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{6, 7, 8}, {3, 4, 5}, {0, 1, 2}}, *output.triangles));
  }

  compression.pruneStoredMesh(101.0);

  mesh = createMesh({block1_empty, block2_empty});
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 105.0);

    CompressionOutput output(compression);
    EXPECT_EQ(9u, output.vertices->points.size());
    EXPECT_EQ(3u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}}, *output.triangles));
  }

  mesh = createMesh({block1_test1});
  {  // limit temporary scopes
    CompressionInputs input;
    // re-integrating an empty block should not add anything
    compression.compressAndIntegrate(
        mesh, input.vertices, input.triangles, input.indices, input.remappings, 105.0);

    CompressionOutput output(compression);
    EXPECT_EQ(15u, output.vertices->points.size());
    EXPECT_EQ(3u, output.invalidated.size());
    EXPECT_EQ(output.vertices->size(), output.timestamps->size());

    EXPECT_TRUE(checkTriangles({{0, 1, 2}, {3, 4, 5}, {9, 10, 11}, {12, 13, 14}},
                               *output.triangles));
  }
}

}  // namespace kimera_pgmo
