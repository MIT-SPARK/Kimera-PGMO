/**
 * @file   test_delta_compression.cpp
 * @brief  Unit-tests for DeltaCompression
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include <gtest/gtest.h>

#include "kimera_pgmo/utils/vertex_update.h"

namespace kimera_pgmo {

TEST(VertexUpdate, DefaultUpdateCorrect) {
  const auto make_traits = [](traits::Timestamp first, traits::Timestamp last) {
    return traits::VertexTraits{
        {true, true, false, true}, {1, 2, 3, 4}, last, 0, first};
  };

  traits::Pos pos;
  traits::VertexTraits traits;
  DefaultVertexUpdate{}(0, traits::Pos(1, 2, 3), make_traits(1, 2), pos, traits);
  EXPECT_TRUE(traits.properties.has_color);
  EXPECT_TRUE(traits.properties.has_stamp);
  EXPECT_TRUE(traits.properties.has_first_seen_stamp);
  EXPECT_EQ(traits.first_seen_stamp, 1u);
  EXPECT_EQ(traits.stamp, 2u);

  DefaultVertexUpdate{}(0, traits::Pos(1, 2, 3), make_traits(3, 4), pos, traits);
  EXPECT_EQ(traits.first_seen_stamp, 1u);
  EXPECT_EQ(traits.stamp, 4u);

  DefaultVertexUpdate{}(0, traits::Pos(1, 2, 3), make_traits(0, 3), pos, traits);
  EXPECT_EQ(traits.first_seen_stamp, 0u);
  EXPECT_EQ(traits.stamp, 4u);
}

}  // namespace kimera_pgmo
