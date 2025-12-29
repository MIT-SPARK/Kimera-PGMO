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
  MeshOffsetInfo::RemapInfo info;

  info.addIndex(5);
  EXPECT_EQ(info.min_index, 5u);
  EXPECT_EQ(info.max_index, 5u);

  info.addIndex(10);
  EXPECT_EQ(info.min_index, 5u);
  EXPECT_EQ(info.max_index, 10u);

  info.addIndex(3);
  EXPECT_EQ(info.min_index, 3u);
  EXPECT_EQ(info.max_index, 10u);
}

}  // namespace kimera_pgmo
