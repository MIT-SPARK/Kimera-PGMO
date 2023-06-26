/**
 * @file   test_delta_compression.cpp
 * @brief  Unit-tests for DeltaCompression
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include "gtest/gtest.h"
#include "kimera_pgmo/compression/DeltaCompression.h"

#include <chrono>

std::ostream& operator<<(std::ostream& out,
                         const std::unordered_map<size_t, size_t>& map) {
  out << "{";

  auto iter = map.begin();
  while (iter != map.end()) {
    out << iter->first << ": " << iter->second;
    ++iter;

    if (iter != map.end()) {
      out << ", ";
    }
  }

  out << "}";
  return out;
}

std::ostream& operator<<(std::ostream& out, const std::map<size_t, size_t>& map) {
  out << "{";

  auto iter = map.begin();
  while (iter != map.end()) {
    out << iter->first << ": " << iter->second;
    ++iter;

    if (iter != map.end()) {
      out << ", ";
    }
  }

  out << "}";
  return out;
}

std::ostream& operator<<(std::ostream& out,
                         const std::vector<kimera_pgmo::Face>& faces) {
  out << "[";

  auto iter = faces.begin();
  while (iter != faces.end()) {
    out << *iter;
    ++iter;

    if (iter != faces.end()) {
      out << ", ";
    }
  }

  out << "]";
  return out;
}

namespace kimera_pgmo {

bool operator==(const Face& lhs, const Face& rhs) {
  std::set<size_t> lset{lhs.v1, lhs.v2, lhs.v3};
  std::set<size_t> rset{rhs.v1, rhs.v2, rhs.v3};
  return lset == rset;
}

bool operator!=(const Face& lhs, const Face& rhs) { return !(lhs == rhs); }

using voxblox::BlockIndex;

struct BlockConfig {
  using FaceCoordinates = std::array<std::array<float, 3>, 3>;

  std::string name;
  std::array<int64_t, 3> index;
  std::vector<FaceCoordinates> faces;
  static uint8_t point_index;

  voxblox_msgs::MeshBlock instantiate() const {
    voxblox_msgs::MeshBlock block;
    std::memcpy(block.index.data(), index.data(), sizeof(block.index));

    // original voxblox conversion: x = (a * \bar{x} + block.x) * L
    // inverse conversion: \bar{x} = 1 / a * (x / L - block.x)
    // for simplicity, block-lenght (L) is 1
    size_t idx = 0;
    for (const auto& face : faces) {
      for (size_t i = 0; i < face.size(); ++i) {
        const float a = std::numeric_limits<uint16_t>::max() / 2.0f;
        block.x.push_back(static_cast<uint16_t>(a * (face[i][0] - index[0])));
        block.y.push_back(static_cast<uint16_t>(a * (face[i][1] - index[1])));
        block.z.push_back(static_cast<uint16_t>(a * (face[i][2] - index[2])));
        block.r.push_back(point_index);
        block.g.push_back(point_index);
        block.b.push_back(point_index);
        ++point_index;
      }
    }

    return block;
  }

  static void resetIndex() { point_index = 0; }
};

uint8_t BlockConfig::point_index = 0;

struct ExpectedDelta {
  size_t vertex_start;
  size_t face_start;
  size_t num_vertices;
  std::vector<Face> expected_triangles;
  VoxbloxIndexMapping expected_remapping;

  void checkOutput(const MeshDelta& output,
                   const VoxbloxIndexMapping& output_remapping,
                   std::map<size_t, size_t>& prev_remapping) const {
    EXPECT_EQ(output.vertex_updates.size(), output.stamp_updates.size())
        << "vertex and timestamp sizes disagree";
    EXPECT_EQ(output.vertex_start, vertex_start);
    EXPECT_EQ(output.face_start, face_start);
    EXPECT_EQ(output.vertex_updates.size(), num_vertices);

    // copy all archived points over from previous delta
    std::map<size_t, size_t> result_remapping;
    for (size_t i = 0; i < output.vertex_start; ++i) {
      if (prev_remapping.count(i)) {
        result_remapping[i] = prev_remapping.at(i);
      }
    }

    // convert current index in delta and absolute index
    for (size_t i = 0; i < output.vertex_updates.size(); ++i) {
      result_remapping[i + output.vertex_start] = output.vertex_updates.at(i).r;
    }

    prev_remapping = result_remapping;

    std::vector<Face> all_output_faces(output.face_archive_updates.begin(),
                                       output.face_archive_updates.end());
    all_output_faces.insert(
        all_output_faces.end(), output.face_updates.begin(), output.face_updates.end());
    checkTriangles(all_output_faces, result_remapping);

    checkRemapping(output_remapping, result_remapping);
  }

  void checkTriangles(const std::vector<Face>& result,
                      const std::map<size_t, size_t>& result_remapping) const {
    EXPECT_EQ(expected_triangles.size(), result.size())
        << "expected: " << expected_triangles << ", result: " << result
        << ", remapping: " << result_remapping;

    std::vector<Face> absolute_faces;
    for (size_t i = 0; i < result.size(); ++i) {
      const auto& rface = result.at(i);
      EXPECT_TRUE(result_remapping.count(rface.v1))
          << rface << " @ " << i << "(map: " << result_remapping << ")";
      EXPECT_TRUE(result_remapping.count(rface.v2))
          << rface << " @ " << i << "(map: " << result_remapping << ")";
      EXPECT_TRUE(result_remapping.count(rface.v3))
          << rface << " @ " << i << "(map: " << result_remapping << ")";
      if (!result_remapping.count(rface.v1) || !result_remapping.count(rface.v2) ||
          !result_remapping.count(rface.v3)) {
        continue;
      }

      Face absolute_face(result_remapping.at(rface.v1),
                         result_remapping.at(rface.v2),
                         result_remapping.at(rface.v3));
      absolute_faces.push_back(absolute_face);

      bool found_match = false;
      for (const auto& expected : expected_triangles) {
        if (expected == absolute_face) {
          found_match = true;
          break;
        }
      }

      EXPECT_TRUE(found_match)
          << "result face "
          << " (r: " << rface << ", a: " << absolute_face << ", i: " << i
          << ") has no match in expected: " << expected_triangles;
    }

    for (size_t i = 0; i < expected_triangles.size(); ++i) {
      const auto& expected = expected_triangles.at(i);

      bool found_match = false;
      for (const auto& absolute_face : absolute_faces) {
        if (expected == absolute_face) {
          found_match = true;
          break;
        }
      }

      EXPECT_TRUE(found_match) << "expected face (r: " << expected << ", i: " << i
                               << ") has no match in result: " << absolute_faces;
    }
  }

  void checkRemapping(const VoxbloxIndexMapping& result,
                      const std::map<size_t, size_t>& result_map) const {
    for (const auto& block_map_pair : expected_remapping) {
      const auto& block = block_map_pair.first;

      EXPECT_TRUE(result.count(block))
          << "missing block " << block.transpose() << " from remapping";
      if (!result.count(block)) {
        continue;
      }

      std::unordered_map<size_t, size_t> absolute_map;
      for (const auto& id_pair : result.at(block)) {
        EXPECT_TRUE(result_map.count(id_pair.second))
            << "v: " << id_pair.first << ", d: " << id_pair.second
            << ", block: " << block.transpose();
        if (!result_map.count(id_pair.second)) {
          break;
        }

        absolute_map[id_pair.first] = result_map.at(id_pair.second);
      }

      EXPECT_EQ(absolute_map, expected_remapping.at(block))
          << "result map " << result.at(block) << " (absolute: " << absolute_map
          << ") does not agree with expected: " << expected_remapping.at(block);
    }

    EXPECT_EQ(result.size(), expected_remapping.size());
  }
};

struct CompressionInput {
  std::optional<std::chrono::nanoseconds> prune_time_ns;
  std::chrono::nanoseconds timestamp_ns;
  std::vector<BlockConfig> blocks;
};

std::ostream& operator<<(std::ostream& out, const CompressionInput& input) {
  out << "prune: ";
  if (input.prune_time_ns) {
    std::chrono::duration<double> prune_time_s = *input.prune_time_ns;
    out << prune_time_s.count() << "[s]";
  } else {
    out << "n/a";
  }

  std::chrono::duration<double> input_time_s = input.timestamp_ns;
  out << ", stamp: " << input_time_s.count() << "[s]";

  out << ", blocks: [";
  auto iter = input.blocks.begin();
  while (iter != input.blocks.end()) {
    out << iter->name;
    ++iter;
    if (iter != input.blocks.end()) {
      out << ",";
    }
  }
  out << "]";

  return out;
}

struct CompressionTestConfiguration {
  std::string name;
  double compression_size;
  std::vector<std::pair<CompressionInput, ExpectedDelta>> inputs;
};

std::ostream& operator<<(std::ostream& out,
                         const CompressionTestConfiguration& config) {
  out << "config " << config.name << " (resolution=" << config.compression_size
      << ") with inputs: " << std::endl;
  for (const auto& input : config.inputs) {
    out << " - input: " << input.first << std::endl;
    ;
    // out << " - expected: " << input.second;
  }
  return out;
}

TEST(test_delta_compression, vertexInfoCorrect) {
  // base info should have ref counts of 0
  VertexInfo info;
  EXPECT_TRUE(info.notObserved());
  EXPECT_TRUE(info.shouldArchive());

  // observation updates both ref counts
  info.addObservation();
  EXPECT_FALSE(info.notObserved());
  EXPECT_FALSE(info.shouldArchive());

  // removing an observation updates both ref counts
  info.removeObservation();
  EXPECT_TRUE(info.notObserved());
  EXPECT_TRUE(info.shouldArchive());

  // archiving an observation only updates a single ref count
  info.addObservation();
  info.archiveObservation();
  EXPECT_FALSE(info.notObserved());
  EXPECT_TRUE(info.shouldArchive());

  // show that removing the last active ref trips the shouldArchive flag
  VertexInfo info2;
  info.addObservation();
  info.addObservation();
  info.archiveObservation();
  EXPECT_FALSE(info.notObserved());
  EXPECT_FALSE(info.shouldArchive());

  info.removeObservation();
  EXPECT_FALSE(info.notObserved());
  EXPECT_TRUE(info.shouldArchive());
}

}  // namespace kimera_pgmo

using namespace std::chrono_literals;
using kimera_pgmo::BlockConfig;
using kimera_pgmo::CompressionTestConfiguration;
using kimera_pgmo::VoxbloxIndexMapping;

struct DeltaCompressionTest
    : public testing::TestWithParam<CompressionTestConfiguration> {};

TEST_P(DeltaCompressionTest, CompressionCorrect) {
  const auto config = GetParam();
  kimera_pgmo::DeltaCompression compression(config.compression_size);

  // reset absoulte index count for vertices
  BlockConfig::resetIndex();

  // keep track of mapping to absolute index
  std::map<size_t, size_t> result_remapping;
  for (const auto& input_expected_pair : config.inputs) {
    const auto& input = input_expected_pair.first;
    const auto& expected = input_expected_pair.second;

    if (input.prune_time_ns) {
      compression.pruneStoredMesh(input.prune_time_ns->count());
    }

    voxblox_msgs::Mesh mesh;
    mesh.block_edge_length = 1.0;
    for (const auto& block : input.blocks) {
      mesh.mesh_blocks.push_back(block.instantiate());
    }

    VoxbloxIndexMapping remapping;
    const auto output =
        compression.update(mesh, input.timestamp_ns.count(), &remapping);
    ASSERT_TRUE(output != nullptr);
    SCOPED_TRACE(input);
    expected.checkOutput(*output, remapping, result_remapping);
  }
}

BlockConfig block1_empty{"block1_empty", {0, 0, 0}, {}};
BlockConfig block1_v1{"block1_v1",
                      {0, 0, 0},
                      {{{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
                       {{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
                       {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};
BlockConfig block1_v2{"block1_v2",
                      {0, 0, 0},
                      {{{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
                       {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

BlockConfig block2_empty{"block2_empty", {-1, 0, 0}, {}};
BlockConfig block2_v1{"block2_v1",
                      {-1, 0, 0},
                      {{{{-0.5, 0.5, 0.5}, {-0.5, 0.75, 0.75}, {-0.5, 0.75, 0.5}}},
                       {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};
BlockConfig block2_v2{"block2_v2",
                      {-1, 0, 0},
                      {{{{-0.5, 0.5, 0.5}, {-0.5, 0.75, 0.75}, {-0.5, 0.75, 0.5}}},
                       {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, -0.5}}}}};

// notes:
// - 0: block1_empty means an empty remapping
// - 1: block1_v1 has one duplicate face (3, 4, 5)
// - 2: archiving block1_v1 and integrating again adds 6 to the first remapping
// - 3: block2_v1 shares a duplicate face with block1_v1
// - 4: block1_empty clears duplicate vertices with block1_v1
// - 5: block2_empty clears duplicate vertices with block2_v1
// - 6: block2_test2 has only unique faces
VoxbloxIndexMapping t1_remappings[] = {
    {{{0, 0, 0},
      {{0, 0}, {1, 1}, {2, 2}, {3, 0}, {4, 1}, {5, 2}, {6, 6}, {7, 7}, {8, 8}}}},
    {{{0, 0, 0}, {}}},
};

VoxbloxIndexMapping t2_remappings[] = {
    {{{0, 0, 0},
      {{0, 0}, {1, 1}, {2, 2}, {3, 0}, {4, 1}, {5, 2}, {6, 6}, {7, 7}, {8, 8}}}},
    {{{0, 0, 0}, {}}},
    {{{0, 0, 0},
      {{0, 9}, {1, 10}, {2, 11}, {3, 9}, {4, 10}, {5, 11}, {6, 15}, {7, 16}, {8, 17}}}},
};

VoxbloxIndexMapping t3_remappings[] = {
    {{{0, 0, 0},
      {{0, 0}, {1, 1}, {2, 2}, {3, 0}, {4, 1}, {5, 2}, {6, 6}, {7, 7}, {8, 8}}},
     {{-1, 0, 0}, {{0, 9}, {1, 10}, {2, 11}, {3, 6}, {4, 7}, {5, 8}}}},
    {{{0, 0, 0}, {}},
     {{-1, 0, 0}, {{0, 15}, {1, 16}, {2, 17}, {3, 18}, {4, 19}, {5, 20}}}},
    {{{0, 0, 0},
      {{0, 21},
       {1, 22},
       {2, 23},
       {3, 21},
       {4, 22},
       {5, 23},
       {6, 27},
       {7, 28},
       {8, 29}}},
     {{-1, 0, 0}, {{0, 30}, {1, 31}, {2, 32}, {3, 27}, {4, 28}, {5, 29}}}},
    {{{0, 0, 0},
      {{0, 36},
       {1, 37},
       {2, 38},
       {3, 36},
       {4, 37},
       {5, 38},
       {6, 42},
       {7, 43},
       {8, 44}}},
     {{-1, 0, 0}, {}}},
};

VoxbloxIndexMapping t4_remappings[] = {
    {{{0, 0, 0},
      {{0, 0}, {1, 1}, {2, 2}, {3, 0}, {4, 1}, {5, 2}, {6, 6}, {7, 7}, {8, 8}}}},
    {{{-1, 0, 0}, {{0, 9}, {1, 10}, {2, 11}, {3, 12}, {4, 13}, {5, 14}}}},
    {{{0, 0, 0}, {}}, {{-1, 0, 0}, {}}},
    {{{0, 0, 0},
      {{0, 15},
       {1, 16},
       {2, 17},
       {3, 15},
       {4, 16},
       {5, 17},
       {6, 21},
       {7, 22},
       {8, 23}}}},
    {{{0, 0, 0},
      {{0, 24},
       {1, 25},
       {2, 26},
       {3, 24},
       {4, 25},
       {5, 26},
       {6, 30},
       {7, 31},
       {8, 32}}}},
};

VoxbloxIndexMapping t5_remappings[] = {
    {{{0, 0, 0},
      {{0, 0}, {1, 1}, {2, 2}, {3, 0}, {4, 1}, {5, 2}, {6, 6}, {7, 7}, {8, 8}}}},
    {{{-1, 0, 0}, {{0, 9}, {1, 10}, {2, 11}, {3, 12}, {4, 13}, {5, 14}}}},
    {{{0, 0, 0}, {}},
     {{-1, 0, 0}, {{0, 15}, {1, 16}, {2, 17}, {3, 18}, {4, 19}, {5, 20}}}},
    {{{-1, 0, 0}, {{0, 21}, {1, 22}, {2, 23}, {3, 24}, {4, 25}, {5, 26}}}},
    {{{0, 0, 0},
      {{0, 27},
       {1, 28},
       {2, 29},
       {3, 27},
       {4, 28},
       {5, 29},
       {6, 33},
       {7, 34},
       {8, 35}}},
     {{-1, 0, 0}, {{0, 36}, {1, 37}, {2, 38}, {3, 33}, {4, 34}, {5, 35}}}},
    {{{0, 0, 0}, {}}, {{-1, 0, 0}, {}}},
};

VoxbloxIndexMapping t6_remappings[] = {
    {{{0, 0, 0}, {{0, 0}, {1, 1}, {2, 2}, {3, 3}, {4, 4}, {5, 5}}}},
    {{{-1, 0, 0}, {{0, 6}, {1, 7}, {2, 8}, {3, 9}, {4, 10}, {5, 11}}}},
    {{{0, 0, 0}, {}},
     {{-1, 0, 0}, {{0, 12}, {1, 13}, {2, 14}, {3, 15}, {4, 16}, {5, 17}}}},
    {{{-1, 0, 0}, {}}},
};

// notes:
// - single_block_clearing:   empty mesh block should clear previous vertices and faces
// - single_block_with_prune: archiving a block before clearing with empty means that
//                            vertices and faces are cached
// - multi_block_clearing:    empty mesh blocks only clear their own vertices and faces
// - multi_block_with_prune:  because of implementation, vertices from block1_v1
//                            get renumbered when block2_v1 gets added
CompressionTestConfiguration test_configurations[] = {
    {"single_block_clearing",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {0, 0, 6, {{0, 1, 2}, {6, 7, 8}}, t1_remappings[0]}},
      {{std::nullopt, 101s, {block1_empty}}, {0, 0, 0, {}, t1_remappings[1]}}}},
    {"single_block_with_prune",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {0, 0, 6, {{0, 1, 2}, {6, 7, 8}}, t2_remappings[0]}},
      {{101s, 102s, {block1_empty}},
       {0, 0, 6, {{0, 1, 2}, {6, 7, 8}}, t2_remappings[1]}},
      {{std::nullopt, 103s, {block1_v1}},
       {6, 2, 6, {{9, 10, 11}, {15, 16, 17}}, t2_remappings[2]}}}},
    {"multi_block_clearing",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1, block2_v1}},
       {0, 0, 9, {{0, 1, 2}, {6, 7, 8}, {9, 10, 11}}, t3_remappings[0]}},
      {{std::nullopt, 101s, {block1_empty, block2_v1}},
       {0, 0, 6, {{15, 16, 17}, {18, 19, 20}}, t3_remappings[1]}},
      {{std::nullopt, 102s, {block1_v1, block2_v1}},
       {0, 0, 9, {{21, 22, 23}, {27, 28, 29}, {30, 31, 32}}, t3_remappings[2]}},
      {{std::nullopt, 103s, {block1_v1, block2_empty}},
       {0, 0, 6, {{36, 37, 38}, {42, 43, 44}}, t3_remappings[3]}}}},
    {"multi_block_with_prune",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {0, 0, 6, {{0, 1, 2}, {6, 7, 8}}, t4_remappings[0]}},
      {{std::nullopt, 102s, {block2_v1}},
       {0, 0, 9, {{0, 1, 2}, {9, 10, 11}, {12, 13, 14}}, t4_remappings[1]}},
      {{101s, 103s, {block1_empty, block2_empty}},
       {0, 0, 6, {{0, 1, 2}, {12, 13, 14}}, t4_remappings[2]}},
      {{std::nullopt, 104s, {block1_v1}},
       {6, 2, 6, {{15, 16, 17}, {21, 22, 23}}, t4_remappings[3]}},
      {{std::nullopt, 105s, {block1_v1}},
       {6, 2, 6, {{24, 25, 26}, {30, 31, 32}}, t4_remappings[4]}}}},
    {"multi_block_with_multiple_partial_updates",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {0, 0, 6, {{0, 1, 2}, {6, 7, 8}}, t5_remappings[0]}},
      {{std::nullopt, 102s, {block2_v1}},
       {0, 0, 9, {{0, 1, 2}, {9, 10, 11}, {12, 13, 14}}, t5_remappings[1]}},
      {{101s, 103s, {block1_empty, block2_v1}},
       {0,
        0,
        9,
        {{0, 1, 2}, {15, 16, 17}, {18, 19, 20}, {18, 19, 20}},
        t5_remappings[2]}},
      {{std::nullopt, 104s, {block2_v1}},
       {3, 1, 6, {{21, 22, 23}, {24, 25, 26}, {24, 25, 26}}, t5_remappings[3]}},
      {{std::nullopt, 105s, {block1_v1, block2_v1}},
       {3,
        1,
        9,
        {{27, 28, 29}, {33, 34, 35}, {36, 37, 38}, {33, 34, 35}},
        t5_remappings[4]}},
      {{std::nullopt, 106s, {block1_empty, block2_empty}},
       {3, 1, 3, {{33, 34, 35}}, t5_remappings[5]}}}},
    {"multi_block_with_partial_archive",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v2}},
       {0, 0, 6, {{0, 1, 2}, {3, 4, 5}}, t6_remappings[0]}},
      {{std::nullopt, 102s, {block2_v2}},
       {0, 0, 10, {{0, 1, 2}, {9, 10, 5}, {6, 7, 8}, {9, 10, 11}}, t6_remappings[1]}},
      {{101s, 103s, {block1_empty, block2_v2}},
       {0,
        0,
        10,
        {{0, 1, 2}, {12, 13, 14}, {15, 16, 17}, {15, 16, 5}},
        t6_remappings[2]}},
      {{std::nullopt, 104s, {block2_empty}},
       {4, 1, 2, {{15, 16, 5}}, t6_remappings[3]}}}},
};

INSTANTIATE_TEST_CASE_P(DeltaCompression,
                        DeltaCompressionTest,
                        testing::ValuesIn(test_configurations));
