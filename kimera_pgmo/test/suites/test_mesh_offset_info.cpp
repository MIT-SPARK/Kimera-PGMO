#include <gtest/gtest.h>

#include "kimera_pgmo/mesh_offset_info.h"

namespace kimera_pgmo {
namespace {

MeshOffsetInfo makeOffsets(size_t curr_archived,
                           size_t prev_archived,
                           size_t archived_faces,
                           const std::map<size_t, size_t>& remapping = {}) {
  return {curr_archived,
          prev_archived,
          archived_faces,
          remapping.empty() ? nullptr
                            : std::make_shared<std::map<size_t, size_t>>(remapping)};
}

}  // namespace

bool operator==(const MeshOffsetInfo::RemapStats& lhs,
                const MeshOffsetInfo::RemapStats& rhs) {
  return lhs.min_index == rhs.min_index && lhs.max_index == rhs.max_index &&
         lhs.all_archived == rhs.all_archived &&
         lhs.deleted_indices == rhs.deleted_indices;
}

TEST(MeshOffsetInfo, RemapTrackingCorrect) {
  MeshOffsetInfo::RemapStats stats;

  stats.addIndex(5);
  EXPECT_EQ(stats.min_index, 5u);
  EXPECT_EQ(stats.max_index, 5u);

  stats.addIndex(10);
  EXPECT_EQ(stats.min_index, 5u);
  EXPECT_EQ(stats.max_index, 10u);

  stats.addIndex(3);
  EXPECT_EQ(stats.min_index, 3u);
  EXPECT_EQ(stats.max_index, 10u);
}

TEST(MeshOffsetInfo, LocalGlobalCorrect) {
  MeshOffsetInfo offsets{5, 2, 10};
  EXPECT_EQ(offsets.toGlobalVertex(0), 2u);
  EXPECT_EQ(offsets.toLocalVertex(2), 0u);

  EXPECT_THROW({ offsets.toLocalVertex(0); }, std::logic_error);
}

TEST(MeshOffsetInfo, RemapVertexCorrect) {
  std::map<size_t, size_t> remap{{0, 2}, {1, 1}, {2, 0}, {4, 3}, {5, 5}, {6, 4}};
  const auto offsets = makeOffsets(8, 5, 2, remap);
  std::vector<std::optional<size_t>> expected{7, 6, 5, std::nullopt, 8, 10, 9};

  // archived indices don't get remapped
  for (size_t i = 0; i < 5; ++i) {
    const auto result = offsets.remapGlobalVertex(i);
    const std::optional<size_t> expected(i);
    EXPECT_EQ(result, expected) << "i: " << i;
  }

  // unarchived indices get remapped
  for (size_t i = 5; i < 12; ++i) {
    const auto result = offsets.remapGlobalVertex(i);
    EXPECT_EQ(result, expected[i - 5]) << "i: " << i;
  }

  const auto no_remap_offsets = makeOffsets(8, 5, 2);
  EXPECT_THROW({ no_remap_offsets.remapGlobalVertex(2); }, std::logic_error);
}

TEST(MeshOffsetInfo, RemapIndicesVectorCorrect) {
  std::map<size_t, size_t> remap{{0, 2}, {1, 1}, {2, 0}, {4, 3}, {5, 5}, {6, 4}};
  const auto offsets = makeOffsets(8, 5, 2, remap);

  const std::vector<size_t> expected{7, 6, 5, 8, 10, 9};
  const MeshOffsetInfo::RemapStats expected_stats{5, 10, false, {3}};

  std::vector<size_t> original{5, 6, 7, 8, 9, 10, 11};
  MeshOffsetInfo::RemapStats result_stats;
  EXPECT_EQ(offsets.remapVertexIndices(original, &result_stats), expected);
  EXPECT_EQ(result_stats, expected_stats);
}

TEST(MeshOffsetInfo, RemapIndicesListCorrect) {
  std::map<size_t, size_t> remap{{0, 2}, {1, 1}, {2, 0}, {4, 3}, {5, 5}, {6, 4}};
  const auto offsets = makeOffsets(8, 5, 2, remap);

  const std::list<size_t> expected{7, 6, 5, 8, 10, 9};
  const MeshOffsetInfo::RemapStats expected_stats{5, 10, false, {3}};

  std::list<size_t> result{5, 6, 7, 8, 9, 10, 11};
  MeshOffsetInfo::RemapStats result_stats;
  offsets.remapVertexIndices(result, &result_stats);
  EXPECT_EQ(result, expected);
  EXPECT_EQ(result_stats, expected_stats);
}

}  // namespace kimera_pgmo
