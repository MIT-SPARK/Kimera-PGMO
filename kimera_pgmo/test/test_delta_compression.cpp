/**
 * @file   test_delta_compression.cpp
 * @brief  Unit-tests for DeltaCompression
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include <gtest/gtest.h>

#include <chrono>

#include "kimera_pgmo/compression/delta_compression.h"
#include "pgmo_fixtures.h"

namespace kimera_pgmo {
namespace {

using BlockConfigs = std::vector<::kimera_pgmo::test::BlockConfig>;

std::string toString(const Face& face) {
  std::stringstream ss;
  ss << face;
  return ss.str();
}

std::string toString(const ::kimera_pgmo::test::BlockConfig& config) {
  return config.name;
}

std::string toString(const std::optional<std::chrono::nanoseconds>& time_ns) {
  if (!time_ns) {
    return "n/a";
  }

  std::chrono::duration<double> time_s = *time_ns;
  return std::to_string(time_s.count()) + "[s]";
}

template <typename K, typename V>
std::string toString(const std::map<K, V>& map) {
  std::stringstream out;
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
  return out.str();
}

template <typename T>
std::string toString(const std::vector<T>& vec) {
  std::stringstream out;
  out << "[";

  auto iter = vec.begin();
  while (iter != vec.end()) {
    out << toString(*iter);
    ++iter;

    if (iter != vec.end()) {
      out << ", ";
    }
  }

  out << "]";
  return out.str();
}

std::vector<Face> remapFaces(const std::vector<Face>& faces,
                             const std::map<size_t, size_t>& remapping) {
  std::vector<Face> to_return;
  for (const auto& face : faces) {
    Face new_face(remapping.at(face.v1), remapping.at(face.v2), remapping.at(face.v3));
    to_return.push_back(new_face);
  }

  return to_return;
}

std::vector<size_t> remapIndices(const std::vector<size_t>& original,
                                 const std::map<size_t, size_t>& remapping) {
  std::vector<size_t> remapped;
  std::transform(original.begin(),
                 original.end(),
                 std::back_inserter(remapped),
                 [&](const auto& idx) { return remapping.at(idx); });
  return remapped;
}

std::vector<size_t> flattenRemapping(const BlockConfigs& order,
                                     const HashedIndexMapping& remapping) {
  std::vector<size_t> flattened;
  for (const auto& block : order) {
    const BlockIndex block_idx(block.index[0], block.index[1], block.index[2]);
    auto iter = remapping.find(block_idx);
    if (iter == remapping.end()) {
      continue;
    }

    std::vector<size_t> block_remapping(iter->second.size());
    for (const auto& [local_idx, global_idx] : iter->second) {
      block_remapping.at(local_idx) = global_idx;
    }

    flattened.insert(flattened.end(), block_remapping.begin(), block_remapping.end());
  }

  return flattened;
}

bool operator==(const Face& lhs, const Face& rhs) {
  std::set<size_t> lset{lhs.v1, lhs.v2, lhs.v3};
  std::set<size_t> rset{rhs.v1, rhs.v2, rhs.v3};
  return lset == rset;
}

}  // namespace

struct CompressionInput {
  std::optional<std::chrono::nanoseconds> prune_time_ns;
  std::chrono::nanoseconds timestamp_ns;
  std::vector<test::BlockConfig> blocks;
};

std::ostream& operator<<(std::ostream& out, const CompressionInput& input) {
  out << "prune: " << toString(input.prune_time_ns)
      << ", stamp: " << toString(input.timestamp_ns)
      << ", blocks: " << toString(input.blocks);
  return out;
}

struct ExpectedState {
  // Expected start index of mesh delta vertices
  size_t vertex_start;
  // Expected start index of mesh delta faces
  size_t face_start;
  // Expected number of vertices in mesh delta
  size_t num_vertices;
};

struct ExpectedDelta {
  // Expected parameters of delta
  ExpectedState state;
  // Expected faces in mesh delta. Indices are specified by absolute input point
  // indices, i.e., where the point falls within the total number of points inserted
  // during the test suite.
  std::vector<Face> expected_triangles;
  // Expected correpondences between input vertices and absolute point indices (input
  // vertex indices are computed by block order, i.e., the first block has indices
  // 0...N, the second block has indices N+1...M and so on).
  std::vector<size_t> expected_indices;

  void checkOutput(const MeshDelta& output,
                   const std::vector<size_t>& output_indices,
                   std::map<size_t, size_t>& prev_remapping) const;

  void checkTriangles(const std::vector<Face>& result,
                      const std::map<size_t, size_t>& result_remapping) const;
};

struct CompressionTestConfiguration {
  std::string name;
  double compression_size;
  std::vector<std::pair<CompressionInput, ExpectedDelta>> inputs;
};

std::ostream& operator<<(std::ostream& out,
                         const CompressionTestConfiguration& config) {
  out << "config " << config.name << " (resolution=" << config.compression_size
      << " with inputs: " << std::endl;
  for (const auto& input : config.inputs) {
    out << " - input: " << input.first << std::endl;
  }
  return out;
}

void ExpectedDelta::checkOutput(const MeshDelta& output,
                                const std::vector<size_t>& output_indices,
                                std::map<size_t, size_t>& prev_remapping) const {
  EXPECT_EQ(output.vertex_updates->size(), output.stamp_updates.size())
      << "vertex and timestamp sizes disagree";
  EXPECT_EQ(output.vertex_start, state.vertex_start);
  EXPECT_EQ(output.face_start, state.face_start);
  EXPECT_EQ(output.vertex_updates->size(), state.num_vertices);

  // copy all archived points over from previous delta
  std::map<size_t, size_t> result_remapping;
  for (size_t i = 0; i < output.vertex_start; ++i) {
    if (prev_remapping.count(i)) {
      result_remapping[i] = prev_remapping.at(i);
    }
  }

  // convert current index in delta and absolute index
  for (size_t i = 0; i < output.vertex_updates->size(); ++i) {
    result_remapping[i + output.vertex_start] = output.vertex_updates->at(i).r;
  }

  prev_remapping = result_remapping;

  std::vector<Face> all_output_faces(output.face_archive_updates.begin(),
                                     output.face_archive_updates.end());
  all_output_faces.insert(
      all_output_faces.end(), output.face_updates.begin(), output.face_updates.end());
  checkTriangles(all_output_faces, result_remapping);

  const auto abs_output_indices = remapIndices(output_indices, result_remapping);
  EXPECT_EQ(expected_indices, abs_output_indices);
}

void ExpectedDelta::checkTriangles(
    const std::vector<Face>& result,
    const std::map<size_t, size_t>& result_remapping) const {
  EXPECT_EQ(expected_triangles.size(), result.size())
      << "expected: " << toString(expected_triangles)
      << ", result: " << toString(remapFaces(result, result_remapping))
      << ", result (original): " << toString(result);

  std::vector<Face> absolute_faces;
  for (size_t i = 0; i < result.size(); ++i) {
    const auto& rface = result.at(i);
    EXPECT_TRUE(result_remapping.count(rface.v1))
        << rface << " @ " << i << "(map: " << toString(result_remapping) << ")";
    EXPECT_TRUE(result_remapping.count(rface.v2))
        << rface << " @ " << i << "(map: " << toString(result_remapping) << ")";
    EXPECT_TRUE(result_remapping.count(rface.v3))
        << rface << " @ " << i << "(map: " << toString(result_remapping) << ")";
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

    EXPECT_TRUE(found_match) << "result face "
                             << " (r: " << rface << ", a: " << absolute_face
                             << ", i: " << i << ") has no match in expected: "
                             << toString(expected_triangles);
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
                             << ") has no match in result: "
                             << toString(absolute_faces);
  }
}

namespace {

using namespace std::chrono_literals;

// contains no vertices or faces and should clear any faces
kimera_pgmo::test::BlockConfig block1_empty{"block1_empty", {0, 0, 0}, {}};

// contains 3 faces, 1 unique to block 1, a duplicate of the first that should always be
// discarded, and 1 shared with block2_v1
kimera_pgmo::test::BlockConfig block1_v1{
    "block1_v1",
    {0, 0, 0},
    {{{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
     {{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
     {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

// contains the same vertices as block1_v1 but drops the repeated first face
kimera_pgmo::test::BlockConfig block1_v2{
    "block1_v2",
    {0, 0, 0},
    {{{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
     {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

// contains no vertices or faces and should clear any faces
kimera_pgmo::test::BlockConfig block2_empty{"block2_empty", {-1, 0, 0}, {}};

// contains 2 faces, 1 unique and 1 shared with block1_v1 and block1_v2
kimera_pgmo::test::BlockConfig block2_v1{
    "block2_v1",
    {-1, 0, 0},
    {{{{-0.5, 0.5, 0.5}, {-0.5, 0.75, 0.75}, {-0.5, 0.75, 0.5}}},
     {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

// contains 2 faces, 1, unique and 1 partially shared with block1_v1 and block1_v2
kimera_pgmo::test::BlockConfig block2_v2{
    "block2_v2",
    {-1, 0, 0},
    {{{{-0.5, 0.5, 0.5}, {-0.5, 0.75, 0.75}, {-0.5, 0.75, 0.5}}},
     {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, -0.5}}}}};

// notes:
// - SingleBlockClearing: Adds block1_v1 and then adds an empty block1 to check that
// vertices and faces are removed
//
// - SingleBlockPrune: Adds block1_v1, archives block1_v1 and then adds an empty block1
// (which should have no result after archiving block1_v1) and then adds block1_v1 which
// should generate new vertices
//
// - MultiBlockClearing: Empty mesh blocks only clear their own vertices and faces
//
// - MultiBlockPrune: Adds block1_v1 and block2_v1 at different times, then
// archives block1 and adds empty blocks for block1 and block2 (effectively clearing
// block 2). Afterwards, updates block1_v1 twice, resulting in new vertices
//
// - MultiBlockPartialUpdates: TBD
// - MultiBlockPartialArchive: TBD
//
// For annotations:
// bX: block X
// fX: face X (1-indexed)
CompressionTestConfiguration test_configurations[] = {
    {"SingleBlockClearing",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {{0, 0, 6},                      // b1 vertices
        {{0, 1, 2}, {6, 7, 8}},         // b1.f2 redundant
        {0, 1, 2, 0, 1, 2, 6, 7, 8}}},  // b1.f2 vertices redundant
      {{std::nullopt, 101s, {block1_empty}}, {{0, 0, 0}, {}, {}}}}},
    {"SingleBlockPrune",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {{0, 0, 6},                      // b1 vertices
        {{0, 1, 2}, {6, 7, 8}},         // b1.f2 redundant
        {0, 1, 2, 0, 1, 2, 6, 7, 8}}},  // b1.f2 vertices redundant
      {{101s, 102s, {block1_empty}},
       {{0, 0, 6},               // b1 vertices archived
        {{0, 1, 2}, {6, 7, 8}},  // b1 non-duplicate faces
        {}}},                    // empty block, no remmaping
      {{std::nullopt, 103s, {block1_v1}},
       {{6, 2, 6},                               // b1 archived plus b1 vertices
        {{9, 10, 11}, {15, 16, 17}},             // b1 faces offset by 9
        {9, 10, 11, 9, 10, 11, 15, 16, 17}}}}},  // b1 vertices offset by 9
    {"MultiBlockClearing",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1, block2_v1}},
       {{0, 0, 9},                                          // b1 and b2 vertices
        {{0, 1, 2}, {6, 7, 8}, {9, 10, 11}},                // b1.f2 and b2.f2 redundant
        {0, 1, 2, 0, 1, 2, 6, 7, 8, 9, 10, 11, 6, 7, 8}}},  // no offset
      {{std::nullopt, 101s, {block1_empty, block2_v1}},
       {{0, 0, 6},                     // b1 cleared: b2 vertices
        {{15, 16, 17}, {18, 19, 20}},  // b2 faces
        {15, 16, 17, 18, 19, 20}}},    // b2 remaps to itself (offset 15)
      {{std::nullopt, 102s, {block1_v1, block2_v1}},
       {{0, 0, 9},                                   // b1 added again: 9 vertices
        {{21, 22, 23}, {27, 28, 29}, {30, 31, 32}},  // b1.f2 and b2.f2 redundant
        {21, 22, 23, 21, 22, 23, 27, 28, 29, 30, 31, 32, 27, 28, 29}}},  // offset 21
      {{std::nullopt, 103s, {block1_v1, block2_empty}},
       {{0, 0, 6},                                 // b2 cleared
        {{36, 37, 38}, {42, 43, 44}},              // both b1 faces
        {36, 37, 38, 36, 37, 38, 42, 43, 44}}}}},  // offset by 36
    {"MultiBlockPrune",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {{0, 0, 6},                      // b1: 6 vertices
        {{0, 1, 2}, {6, 7, 8}},         // b1: 2 faces
        {0, 1, 2, 0, 1, 2, 6, 7, 8}}},  // normal remapping for b1
      {{std::nullopt, 102s, {block2_v1}},
       {{0, 0, 9},                               // b2: 9 vertices
        {{0, 1, 2}, {9, 10, 11}, {12, 13, 14}},  // b1 + b2 faces (no b1 offset)
        {9, 10, 11, 12, 13, 14}}},  // b2 vertices override previous clusters
      {{101s, 103s, {block1_empty, block2_empty}},
       {{0, 0, 6},                  // b1 archived
        {{0, 1, 2}, {12, 13, 14}},  // b1 archived faces
        {}}},                       // no remapping for empty input
      {{std::nullopt, 104s, {block1_v1}},
       {{6, 2, 6},                               // b1 added again
        {{15, 16, 17}, {21, 22, 23}},            // b1 faces
        {15, 16, 17, 15, 16, 17, 21, 22, 23}}},  // offset 15
      {{std::nullopt, 105s, {block1_v1}},
       {{6, 2, 6},                                 // b1 stays added
        {{24, 25, 26}, {30, 31, 32}},              // faces indices overriden
        {24, 25, 26, 24, 25, 26, 30, 31, 32}}}}},  // offset 24
    {"MultiBlockPartialUpdates",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {{0, 0, 6},                      // b1 vertices
        {{0, 1, 2}, {6, 7, 8}},         // b1.f2 duplicate
        {0, 1, 2, 0, 1, 2, 6, 7, 8}}},  // no offset
      {{std::nullopt, 102s, {block2_v1}},
       {{0, 0, 9},                               // b1 + b2 vertices
        {{0, 1, 2}, {9, 10, 11}, {12, 13, 14}},  // b1.f2 and b2.f2 duplicate
        {9, 10, 11, 12, 13, 14}}},               // b2 with offset 9
      {{101s, 103s, {block1_empty, block2_v1}},
       {{0, 0, 9},                                              // b1 archive
        {{0, 1, 2}, {15, 16, 17}, {18, 19, 20}, {18, 19, 20}},  // b2.f2 duplicate
                                                                // with archived
                                                                // b1.f2
        {15, 16, 17, 18, 19, 20}}},                             // b2 with offset 15
      {{std::nullopt, 104s, {block2_v1}},
       {{3, 1, 6},                                   // b2 re-added
        {{21, 22, 23}, {24, 25, 26}, {24, 25, 26}},  // b2.f2 duplicate with b1.f2
        {21, 22, 23, 24, 25, 26}}},                  // b2 with offset 21
      {{std::nullopt, 105s, {block1_v1, block2_v1}},
       {{3, 1, 9},                                                 // b1+b2
        {{27, 28, 29}, {33, 34, 35}, {36, 37, 38}, {33, 34, 35}},  // b2.f2 + b1.f2
        {27, 28, 29, 27, 28, 29, 33, 34, 35, 36, 37, 38, 33, 34, 35}}},  // offset 27
                                                                         // b1 + b2
      {{std::nullopt, 106s, {block1_empty, block2_empty}},
       {{3, 1, 3},       // back to empty
        {{33, 34, 35}},  // partial archive finalized
        {}}}}},          // no remapping
    {"MultiBlockPartialArchive",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v2}},
       {{0, 0, 6},               // b1 (v2)
        {{0, 1, 2}, {3, 4, 5}},  // both faces
        {0, 1, 2, 3, 4, 5}}},
      {{std::nullopt, 102s, {block2_v2}},
       {{0, 0, 10},                                       // b2 (v2): 1 extra vertex
        {{0, 1, 2}, {9, 10, 5}, {6, 7, 8}, {9, 10, 11}},  // all faces, b2v2 overrides
        {6, 7, 8, 9, 10, 11}}},                           // b2 only remapping
      {{101s, 103s, {block1_empty, block2_v2}},
       {{0, 0, 10},                                            // b1 archived + b2v2
        {{0, 1, 2}, {12, 13, 14}, {15, 16, 17}, {15, 16, 5}},  // all faces
        {12, 13, 14, 15, 16, 17}}},                            // b2v2 with 12 offset
      {{std::nullopt, 104s, {block2_empty}},
       {{4, 1, 2},      // 2 shared vertices with b2 get archived this pass
        {{15, 16, 5}},  // partial face
        {}}}}},         // no remapping
    {"RepeatedTimestamp",
     1.0e-3,
     {{{std::nullopt, 100s, {block1_v1}},
       {{0, 0, 6},                      // b1 vertices
        {{0, 1, 2}, {6, 7, 8}},         // b1.f2 redundant
        {0, 1, 2, 0, 1, 2, 6, 7, 8}}},  // no offset
      {{std::nullopt, 100s, {block2_v1}},
       {{0, 0, 9},                               // b2 vertices added
        {{0, 1, 2}, {9, 10, 11}, {12, 13, 14}},  // b1.f1, b1.f3, b2.f1
        {9, 10, 11, 12, 13, 14}}}}},             // b2 remaps to itself (offset 9)
};

}  // namespace

TEST(TestDeltaCompression, vertexInfoCorrect) {
  // base info should have a ref count of 0
  VertexInfo info;
  EXPECT_TRUE(info.notObserved());
  EXPECT_FALSE(info.shouldArchive());

  // observation updates the ref count
  info.addObservation();
  EXPECT_FALSE(info.notObserved());
  EXPECT_FALSE(info.shouldArchive());

  // removing an observation reverts the ref count
  info.removeObservation();
  EXPECT_TRUE(info.notObserved());
  EXPECT_FALSE(info.shouldArchive());

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

struct DeltaCompressionFixture
    : public testing::TestWithParam<CompressionTestConfiguration> {};

INSTANTIATE_TEST_SUITE_P(
    TestDeltaCompression,
    DeltaCompressionFixture,
    testing::ValuesIn(test_configurations),
    [](const testing::TestParamInfo<DeltaCompressionFixture::ParamType>& info) {
      return info.param.name;
    });

TEST_P(DeltaCompressionFixture, CompressionCorrect) {
  const auto config = GetParam();
  DeltaCompression compression(config.compression_size);

  // reset absoulte index count for vertices
  ::kimera_pgmo::test::BlockConfig::resetIndex();

  // keep track of mapping to absolute index
  std::map<size_t, size_t> result_remapping;
  for (const auto& [input, expected] : config.inputs) {
    if (input.prune_time_ns) {
      compression.archiveBlocksByTime(input.prune_time_ns->count());
    }

    auto mesh = ::kimera_pgmo::test::createMesh(input.blocks);
    ASSERT_TRUE(mesh);

    HashedIndexMapping remapping;
    const auto output =
        compression.update(*mesh, input.timestamp_ns.count(), &remapping);
    ASSERT_TRUE(output != nullptr);

    const auto result_indices = flattenRemapping(input.blocks, remapping);

    SCOPED_TRACE(input);
    expected.checkOutput(*output, result_indices, result_remapping);
  }
}

}  // namespace kimera_pgmo
