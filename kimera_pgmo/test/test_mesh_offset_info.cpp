#include <gtest/gtest.h>

#include "kimera_pgmo/mesh_offset_info.h"

namespace kimera_pgmo {

TEST(MeshOffsetInfo, LocalGlobalCorrect) {
  MeshOffsetInfo offsets{5, 2, 10, 5};
  EXPECT_EQ(offsets.toGlobalVertex(0), 2u);
  EXPECT_EQ(offsets.toLocalVertex(2), 0u);

  EXPECT_THROW({ offsets.toLocalVertex(0); }, std::logic_error);
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

}  // namespace kimera_pgmo
