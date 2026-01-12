/**
 * @file   test_delta_compression.cpp
 * @brief  Unit-tests for DeltaCompression
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include <gtest/gtest.h>

#include <chrono>
#include <numeric>
#include <sstream>
#include <string>

#include "kimera_pgmo/compression/delta_compression.h"
#include "kimera_pgmo/mesh_delta.h"
#include "kimera_pgmo/mesh_traits.h"  // IWYU pragma: keep
#include "pgmo_fixtures.h"

namespace kimera_pgmo {

struct TestMesh {
  std::vector<traits::Vertex> points;
} mesh;

size_t pgmoNumVertices(const TestMesh& mesh) {
  return ::kimera_pgmo::pgmoNumVertices(mesh.points);
}

traits::VertexProperties pgmoGetVertexProperties(const TestMesh& mesh) {
  return ::kimera_pgmo::pgmoGetVertexProperties(mesh.points);
}

traits::Pos pgmoGetVertex(const TestMesh& mesh,
                          size_t i,
                          traits::VertexTraits* traits = nullptr) {
  return ::kimera_pgmo::pgmoGetVertex(mesh.points, i, traits);
}

size_t pgmoNumFaces(const TestMesh& faces) { return 0; }

traits::Face pgmoGetFace(const TestMesh& faces, size_t i) { return {}; }

namespace {

using ::kimera_pgmo::test::MeshBlock;
using traits::Face;

using OrderedMesh = std::vector<std::pair<BlockIndex, MeshBlock>>;
using BlockConfigs = std::vector<::kimera_pgmo::test::BlockConfig>;
using Faces = std::vector<traits::Face>;

OrderedMesh createMesh(const BlockConfigs& configs) {
  OrderedMesh mesh;
  for (const auto& config : configs) {
    const BlockIndex block_idx(config.index[0], config.index[1], config.index[2]);
    MeshBlock block(1.0, block_idx);
    config.fillBlock(block);
    mesh.push_back({block_idx, block});
  }

  return mesh;
}

std::string toString(size_t index) { return std::to_string(index); }

std::string toString(const Face& face) {
  std::stringstream ss;
  ss << "(" << face[0] << ", " << face[1] << ", " << face[2] << ")";
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

Face remapFace(const Face& face, const std::vector<size_t>& remapping) {
  return {remapping.at(face[0]), remapping.at(face[1]), remapping.at(face[2])};
}

Faces remapFaces(const Faces& faces, const std::vector<size_t>& remapping) {
  Faces to_return;
  for (const auto& face : faces) {
    to_return.push_back(remapFace(face, remapping));
  }

  return to_return;
}

std::vector<size_t> remapIndices(const std::vector<size_t>& original,
                                 const std::vector<size_t>& remapping) {
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

bool sameFace(const Face& lhs, const Face& rhs) {
  std::set<size_t> lset{lhs[0], lhs[1], lhs[2]};
  std::set<size_t> rset{rhs[0], rhs[1], rhs[2]};
  return lset == rset;
}

bool faceInFaces(const Face& face, const Faces& faces) {
  for (const auto& f : faces) {
    if (sameFace(face, f)) {
      return true;
    }
  }

  return false;
}

Faces facesFromDelta(const MeshDelta& delta) {
  Faces faces;
  const auto& archived = delta.archived_faces();
  faces.insert(faces.end(), archived.begin(), archived.end());
  const auto& active = delta.faces();
  faces.insert(faces.end(), active.begin(), active.end());
  return faces;
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
  // Expected number of vertices in mesh delta
  size_t num_vertices;
  // Expected number of active vertices in the last delta
  size_t prev_active_vertices;
  // Expected number of active faces in the last delta
  size_t prev_active_faces;
  // Expected total archived vertices
  size_t archived_vertices;
  // Expected total archived faces
  size_t archived_faces;
};

struct ExpectedDelta {
  // Expected parameters of delta
  ExpectedState state;
  // Expected faces in mesh delta. Indices are specified by absolute input point
  // indices, i.e., where the point falls within the total number of points inserted
  // during the test suite.
  Faces expected_triangles;
  // Expected correpondences between input vertices and absolute point indices (input
  // vertex indices are computed by block order, i.e., the first block has indices
  // 0...N, the second block has indices N+1...M and so on).
  std::vector<size_t> expected_indices;

  void checkOutput(const MeshDelta& output,
                   const std::vector<size_t>& output_indices) const;

  void checkTriangles(const Faces& result, const std::vector<size_t>& remapping) const;

  void checkMesh(const MeshOffsetInfo& offsets,
                 const std::vector<traits::Vertex>& prev_vertices,
                 const std::vector<traits::Vertex>& vertices,
                 const std::vector<traits::Face>& faces) const;
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
                                const std::vector<size_t>& output_indices) const {
  EXPECT_EQ(output.info.prev_active_vertices, state.prev_active_vertices);
  EXPECT_EQ(output.info.prev_active_faces, state.prev_active_faces);
  EXPECT_EQ(output.getNumVertices(), state.num_vertices);

  // convert current index in delta to absolute index
  std::vector<size_t> remapping;
  for (size_t i = 0; i < output.getNumVertices(); ++i) {
    const auto& v = output.getVertex(i);
    remapping.push_back(v.traits.color.at(0));
  }

  const auto all_output_faces = facesFromDelta(output);
  checkTriangles(all_output_faces, remapping);

  const auto abs_output_indices = remapIndices(output_indices, remapping);
  EXPECT_EQ(expected_indices, abs_output_indices);
}

void ExpectedDelta::checkTriangles(const Faces& result,
                                   const std::vector<size_t>& remapping) const {
  EXPECT_EQ(expected_triangles.size(), result.size())
      << "expected: " << toString(expected_triangles)
      << ", result: " << toString(remapFaces(result, remapping))
      << ", result (original): " << toString(result);

  std::vector<Face> absolute_faces;
  for (size_t i = 0; i < result.size(); ++i) {
    const auto& rface = result.at(i);
    std::stringstream ss;
    ss << toString(rface) << " @ " << i << "(map: " << toString(remapping) << ")";

    EXPECT_LT(rface[0], remapping.size()) << ss.str();
    EXPECT_LT(rface[1], remapping.size()) << ss.str();
    EXPECT_LT(rface[2], remapping.size()) << ss.str();
    if (remapping.size() <= rface[0] || remapping.size() <= rface[1] ||
        remapping.size() <= rface[2]) {
      continue;
    }

    const auto aface = remapFace(rface, remapping);
    absolute_faces.push_back(aface);

    const auto found_match = faceInFaces(aface, expected_triangles);
    EXPECT_TRUE(found_match) << "result face "
                             << " (r: " << toString(rface) << ", a: " << toString(aface)
                             << ", i: " << i << ") has no match in expected: "
                             << toString(expected_triangles) << " with remapping "
                             << toString(remapping);
  }

  for (size_t i = 0; i < expected_triangles.size(); ++i) {
    const auto& expected = expected_triangles.at(i);
    const auto found_match = faceInFaces(expected, absolute_faces);
    EXPECT_TRUE(found_match) << "expected face (r: " << toString(expected)
                             << ", i: " << i
                             << ") has no match in result: " << toString(absolute_faces)
                             << " with remapping " << toString(remapping);
  }
}

void ExpectedDelta::checkMesh(const MeshOffsetInfo& offsets,
                              const std::vector<traits::Vertex>& prev_vertices,
                              const std::vector<traits::Vertex>& vertices,
                              const std::vector<traits::Face>& /* faces */) const {
  EXPECT_EQ(state.archived_vertices, offsets.archived_vertices);
  EXPECT_EQ(state.archived_faces, offsets.archived_faces);

  // we want to map the original index in the last delta to the new index
  // the original indices are in [prev_archived, prev_remapped + prev_archived)
  ASSERT_LE(offsets.prev_archived_vertices, prev_vertices.size());
  const auto prev_remapped = prev_vertices.size() - offsets.prev_archived_vertices;
  std::list<size_t> prev_indices(prev_remapped);
  std::iota(prev_indices.begin(), prev_indices.end(), offsets.prev_archived_vertices);

  MeshOffsetInfo::RemapStats stats;
  offsets.remapVertexIndices(prev_indices, &stats);

  auto iter = prev_indices.begin();
  std::map<size_t, size_t> result_remap;
  for (size_t i = 0; i < prev_remapped; ++i) {
    if (stats.deleted_indices.count(i)) {
      continue;
    }

    // we associate any previous index with the new one if it wasn't deleted
    ASSERT_NE(iter, prev_indices.end());
    result_remap[i + offsets.prev_archived_vertices] = *iter;
    ++iter;
  }

  for (const auto& [prev, curr] : result_remap) {
    ASSERT_LT(prev, prev_vertices.size());
    ASSERT_LT(curr, vertices.size());
    const auto p_prev = prev_vertices.at(prev).pos;
    const auto p_curr = vertices.at(curr).pos;
    Eigen::IOFormat fmt(3, Eigen::DontAlignCols, ", ", "; ", "", "", "[", "]");
    EXPECT_NEAR((p_prev - p_curr).norm(), 0.0, 1.0e-6)
        << "prev: " << p_prev.format(fmt) << ", curr: " << p_curr.format(fmt)
        << ", remap: " << prev << " -> " << curr;
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

// contains the same vertices as block1_v1 but with a face that has the boundary vertex
// with b2v2
kimera_pgmo::test::BlockConfig block1_v3{
    "block1_v3",
    {0, 0, 0},
    {
        {{{0.5, 0.5, 0.5}, {0.5, 0.75, 0.75}, {0.5, 0.75, 0.5}}},
        {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}},
        {{{0.1, 0.2, 0.3}, {0.2, 0.3, 0.4}, {0.0, 0.0, 0.5}}},
    }};

// contains no vertices or faces and should clear any faces
kimera_pgmo::test::BlockConfig block2_empty{"block2_empty", {-1, 0, 0}, {}};

// contains 2 faces, 1 unique and 1 shared with block1_v1 and block1_v2
kimera_pgmo::test::BlockConfig block2_v1{
    "block2_v1",
    {-1, 0, 0},
    {{{{-0.5, 0.5, 0.5}, {-0.5, 0.75, 0.75}, {-0.5, 0.75, 0.5}}},
     {{{0.0, 0.0, 0.0}, {0.0, 0.5, 0.5}, {0.0, 0.0, 0.5}}}}};

// contains 2 faces, 1 unique and 1 partially shared with block1_v1 and block1_v2
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
     {
         {{std::nullopt, 100s, {block1_v1}},
          {{6, 0, 0, 0, 0},  // 6 unique vertices
           {{3, 4, 5}, {6, 7, 8}},
           {3, 4, 5, 3, 4, 5, 6, 7, 8}}},
         {{std::nullopt, 101s, {block1_empty}},
          {{0, 6, 2, 0, 0},  // empty -> clear 6 vertices and 2 faces
           {},
           {}}},
     }},
    {"SingleBlockPrune",
     1.0e-3,
     {
         {{std::nullopt, 100s, {block1_v1}},
          {
              {6, 0, 0, 0, 0},  // b1 has 6 vertices
              {{3, 4, 5}, {6, 7, 8}},
              {3, 4, 5, 3, 4, 5, 6, 7, 8},
          }},
         {{101s, 102s, {block1_empty}},
          {
              {6, 6, 2, 6, 2},  // replacing previous 6 with archived 6
              {{3, 4, 5}, {6, 7, 8}},
              {},
          }},
         {{std::nullopt, 103s, {block1_v1}},
          {
              {6, 0, 0, 6, 2},  // new 6 vertices + archival
              {{12, 13, 14}, {15, 16, 17}},
              {12, 13, 14, 12, 13, 14, 15, 16, 17},
          }},
     }},
    {"MultiBlockClearing",
     1.0e-3,
     {
         {{std::nullopt, 100s, {block1_v1, block2_v1}},
          {
              {9, 0, 0, 0, 0},  // 9 unique vertices (and 3 faces)
              {{3, 4, 5}, {12, 13, 14}, {9, 10, 11}},
              {3, 4, 5, 3, 4, 5, 12, 13, 14, 9, 10, 11, 12, 13, 14},
          }},
         {{std::nullopt, 101s, {block1_empty, block2_v1}},
          {
              {6, 9, 3, 0, 0},  // 6 unique vertices, clear 9/3 previous
              {{15, 16, 17}, {18, 19, 20}},
              {15, 16, 17, 18, 19, 20},
          }},
         {{std::nullopt, 102s, {block1_v1, block2_v1}},
          {
              {9, 6, 2, 0, 0},  // 9/3 unique, clear 6/2 previous
              {{24, 25, 26}, {33, 34, 35}, {30, 31, 32}},
              {24, 25, 26, 24, 25, 26, 33, 34, 35, 30, 31, 32, 33, 34, 35},
          }},
         {{std::nullopt, 103s, {block1_v1, block2_empty}},
          {
              {6, 9, 3, 0, 0},  // 6/2 unique, clear 9/3 previous
              {{39, 40, 41}, {42, 43, 44}},
              {39, 40, 41, 39, 40, 41, 42, 43, 44},
          }},
     }},
    {"MultiBlockPrune",
     1.0e-3,
     {
         {{std::nullopt, 100s, {block1_v1}},
          {
              {6, 0, 0, 0, 0},  // 6/2 unique
              {{3, 4, 5}, {6, 7, 8}},
              {3, 4, 5, 3, 4, 5, 6, 7, 8},
          }},
         {{std::nullopt, 102s, {block2_v1}},
          {
              {9, 6, 2, 0, 0},  // 9/3 unique, remove 6/2
              {{3, 4, 5}, {12, 13, 14}, {9, 10, 11}},
              {9, 10, 11, 12, 13, 14},
          }},
         {{101s, 103s, {block1_empty, block2_empty}},
          {
              {6, 9, 3, 3, 1},  //  6/2 unique, remove 9/3
              {{3, 4, 5}, {12, 13, 14}},
              {},
          }},
         {{std::nullopt, 104s, {block1_v1}},
          {
              {9, 3, 0, 3, 1},  // 6/2 unique, archive 3/1, 3/1 pending
              {{18, 19, 20}, {21, 22, 23}},
              {18, 19, 20, 18, 19, 20, 21, 22, 23},
          }},
         {{std::nullopt, 105s, {block1_v1}},
          {
              {9, 9, 2, 3, 1},  // 6/2 unique, archive 3/1, 3/1 pending
              {{27, 28, 29}, {30, 31, 32}},
              {27, 28, 29, 27, 28, 29, 30, 31, 32},
          }},
     }},
    {"MultiBlockPartialUpdates",
     1.0e-3,
     {
         {{std::nullopt, 100s, {block1_v1}},
          {
              {6, 0, 0, 0, 0},  // 6/2 unique
              {{3, 4, 5}, {6, 7, 8}},
              {3, 4, 5, 3, 4, 5, 6, 7, 8},
          }},
         {{std::nullopt, 102s, {block2_v1}},
          {
              {9, 6, 2, 0, 0},  // 9/3 unique, remove 6/2
              {{3, 4, 5}, {12, 13, 14}, {9, 10, 11}},
              {9, 10, 11, 12, 13, 14},
          }},
         {{101s, 103s, {block1_empty, block2_v1}},
          {
              {9, 9, 3, 3, 1},  // 9/3 unique, remove 9/3
              {{3, 4, 5}, {18, 19, 20}, {15, 16, 17}, {18, 19, 20}},
              {15, 16, 17, 18, 19, 20},
          }},
         {{std::nullopt, 104s, {block2_v1}},
          {
              {6, 6, 3, 3, 1},  // 3/2 unique, archive 3/1, pending 3/1
              {{24, 25, 26}, {21, 22, 23}, {24, 25, 26}},
              {21, 22, 23, 24, 25, 26},
          }},
         {{std::nullopt, 105s, {block1_v1, block2_v1}},
          {
              {9, 6, 3, 3, 1},  // 6/2 unique, pending 3/1
              {{39, 40, 41}, {30, 31, 32}, {36, 37, 38}, {39, 40, 41}},
              {30, 31, 32, 30, 31, 32, 39, 40, 41, 36, 37, 38, 39, 40, 41},
          }},
         {{std::nullopt, 106s, {block1_empty, block2_empty}},
          {
              {3, 9, 4, 3, 1},  // 6/2 unique, pending 3/1
              {{39, 40, 41}},
              {},
          }},
     }},
    {"MultiBlockPartialArchive",
     1.0e-3,
     {
         {{std::nullopt, 100s, {block1_v2}},
          {
              {6, 0, 0, 0, 0},  // 6/2 unique
              {{0, 1, 2}, {3, 4, 5}},
              {0, 1, 2, 3, 4, 5},
          }},
         {{std::nullopt, 102s, {block2_v2}},
          {
              {10, 6, 2, 0, 0},  // 10/4 unique
              {{0, 1, 2}, {9, 10, 5}, {6, 7, 8}, {9, 10, 11}},
              {6, 7, 8, 9, 10, 11},
          }},
         {{101s, 103s, {block1_empty, block2_v2}},
          {
              {10, 10, 4, 3, 1},  // 10/4 unique, replace all
              {{0, 1, 2}, {15, 16, 5}, {12, 13, 14}, {15, 16, 17}},
              {12, 13, 14, 15, 16, 17},
          }},
         {{std::nullopt, 104s, {block2_empty}},
          {
              {3, 7, 3, 3, 1},  // 3/1 pending, remove 7/3
              {{15, 16, 5}},
              {},
          }},
     }},
    {"BoundaryVertexRemapping",
     1.0e-3,
     {
         {{std::nullopt, 100s, {block1_v3}},
          {
              {8, 0, 0, 0, 0},  // 8/3 unique
              {{0, 1, 2}, {3, 4, 8}, {6, 7, 8}},
              {0, 1, 2, 3, 4, 8, 6, 7, 8},
          }},
         {{std::nullopt, 102s, {block2_v2}},
          {
              {12, 8, 3, 0, 0},  // 12/5 unique, previous 8/3
              {{0, 1, 2}, {12, 13, 8}, {6, 7, 8}, {9, 10, 11}, {12, 13, 14}},
              {9, 10, 11, 12, 13, 14},
          }},
         {{101s, 103s, {block1_empty, block2_v2}},
          {
              {12, 12, 5, 3, 1},  // 12/5 unique, previous 12/5
              {{0, 1, 2}, {18, 19, 8}, {6, 7, 8}, {15, 16, 17}, {18, 19, 20}},
              {15, 16, 17, 18, 19, 20},
          }},
         {{std::nullopt, 104s, {block2_empty}},
          {
              {5, 9, 3, 3, 1},  // 3/1 unique, previous 12/5, archive 3/2
              {{18, 19, 8}},
              {},
          }},
         {{std::nullopt, 105s, {block2_empty}},
          {
              {5, 5, 0, 3, 1},  // 3/1 unique, previous 3/1
              {},
              {},
          }},
     }},
    {"RepeatedTimestamp",
     1.0e-3,
     {
         {{std::nullopt, 100s, {block1_v1}},
          {
              {6, 0, 0, 0, 0},  // 6/2 unique
              {{3, 4, 5}, {6, 7, 8}},
              {3, 4, 5, 3, 4, 5, 6, 7, 8},
          }},
         {{std::nullopt, 100s, {block2_v1}},
          {
              {9, 6, 2, 0, 0},  // 9/3 unique, 6/2 prev
              {{3, 4, 5}, {9, 10, 11}, {12, 13, 14}},
              {9, 10, 11, 12, 13, 14},
          }},
     }},
};

}  // namespace

TEST(DeltaCompression, VertexInfoCorrect) {
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

TEST(DeltaCompression, MergeOperatorCorrect) {
  const auto make_traits = [](traits::Timestamp first, traits::Timestamp last) {
    return traits::VertexTraits{
        {true, true, false, true}, {1, 2, 3, 4}, last, 0, first};
  };

  VertexInfo info;
  DefaultVertexUpdate{}(0, traits::Pos(1, 2, 3), make_traits(1, 2), info);
  EXPECT_TRUE(info.traits.properties.has_color);
  EXPECT_TRUE(info.traits.properties.has_stamp);
  EXPECT_TRUE(info.traits.properties.has_first_seen_stamp);
  EXPECT_EQ(info.traits.first_seen_stamp, 1u);
  EXPECT_EQ(info.traits.stamp, 2u);

  DefaultVertexUpdate{}(0, traits::Pos(1, 2, 3), make_traits(3, 4), info);
  EXPECT_EQ(info.traits.first_seen_stamp, 1u);
  EXPECT_EQ(info.traits.stamp, 4u);

  DefaultVertexUpdate{}(0, traits::Pos(1, 2, 3), make_traits(0, 3), info);
  EXPECT_EQ(info.traits.first_seen_stamp, 0u);
  EXPECT_EQ(info.traits.stamp, 4u);
}

TEST(DeltaCompression, UpdateWithMergeCorrect) {
  const auto make_traits =
      [](traits::Timestamp first, traits::Timestamp last, traits::Label label = 0) {
        return traits::VertexTraits{
            {true, true, label > 0, true}, {1, 2, 3, 4}, last, label, first};
      };

  mesh.points.push_back({traits::Pos(1, 2, 3), make_traits(1, 2)});
  mesh.points.push_back({traits::Pos(1, 2, 3), make_traits(3, 4, 5)});
  mesh.points.push_back({traits::Pos(1, 2, 3), make_traits(0, 3)});

  std::vector<std::pair<BlockIndex, TestMesh>> blocks{{{0, 0, 0}, mesh}};
  DeltaCompression compression(2.0);
  const auto delta = compression.update(blocks, 0);
  ASSERT_TRUE(delta);
  ASSERT_EQ(delta->getNumVertices(), 1);
  const auto& v = delta->getVertex(0);
  EXPECT_EQ(v.pos.x(), 1.0);
  EXPECT_EQ(v.pos.y(), 2.0);
  EXPECT_EQ(v.pos.z(), 3.0);
  EXPECT_TRUE(v.traits.properties.has_color);
  EXPECT_TRUE(v.traits.properties.has_stamp);
  EXPECT_TRUE(v.traits.properties.has_label);
  EXPECT_TRUE(v.traits.properties.has_first_seen_stamp);
  EXPECT_EQ(v.traits.color[0], 1);
  EXPECT_EQ(v.traits.color[1], 2);
  EXPECT_EQ(v.traits.color[2], 3);
  EXPECT_EQ(v.traits.color[3], 4);
  EXPECT_EQ(v.traits.stamp, 4);
  EXPECT_EQ(v.traits.label, 5);
  EXPECT_EQ(v.traits.first_seen_stamp, 0);
}

TEST(DeltaCompression, InvalidFacesCorrect) {
  // 2m resolution means every vertex collapses to the same vertex
  DeltaCompression compression(2.0);
  auto mesh = createMesh({block1_v1});
  const auto output = compression.update(mesh, 0);

  ASSERT_TRUE(output);
  EXPECT_EQ(output->getNumVertices(), 1);
  EXPECT_EQ(output->getNumFaces(), 0);  // all faces should be dropped as invalid
}

struct DeltaCompressionFixture
    : public testing::TestWithParam<CompressionTestConfiguration> {};

INSTANTIATE_TEST_SUITE_P(
    DeltaCompression,
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

  MeshOffsetInfo offsets;
  std::vector<traits::Face> result_faces;
  std::vector<traits::Vertex> result_vertices;
  std::vector<traits::Vertex> prev_vertices;
  for (const auto& [input, expected] : config.inputs) {
    if (input.prune_time_ns) {
      compression.archiveBlocksByTime(input.prune_time_ns->count());
    }

    const auto timestamp_ns = input.timestamp_ns.count();
    auto mesh = createMesh(input.blocks);

    HashedIndexMapping remapping;
    const auto output = compression.update(mesh, timestamp_ns, &remapping);
    ASSERT_TRUE(output != nullptr);

    SCOPED_TRACE(input);
    const auto result_indices = flattenRemapping(input.blocks, remapping);
    expected.checkOutput(*output, result_indices);

    prev_vertices = result_vertices;
    output->updateMesh(result_vertices, result_faces, offsets);
    expected.checkMesh(offsets, prev_vertices, result_vertices, result_faces);
  }
}

}  // namespace kimera_pgmo
