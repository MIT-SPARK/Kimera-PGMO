#include <gtest/gtest.h>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo::traits {

TEST(MeshTypes, PropertiesAndCorrect) {
  VertexProperties original{true, true, true, true};

  VertexProperties no_color{false, true, true, true};
  original &= no_color;
  VertexProperties expected{false, true, true, true};
  EXPECT_EQ(original, expected);

  VertexProperties no_stamp{true, false, true, true};
  original &= no_stamp;
  expected = {false, false, true, true};
  EXPECT_EQ(original, expected);

  VertexProperties no_label{true, true, false, true};
  original &= no_label;
  expected = {false, false, false, true};
  EXPECT_EQ(original, expected);

  VertexProperties no_first_seen{true, true, true, false};
  original &= no_first_seen;
  expected = {false, false, false, false};
  EXPECT_EQ(original, expected);
}

TEST(MeshTypes, PropertiesOrCorrect) {
  VertexProperties original{false, false, false, false};

  VertexProperties has_color{true, false, false, false};
  original |= has_color;
  VertexProperties expected{true, false, false, false};
  EXPECT_EQ(original, expected);

  VertexProperties has_stamp{false, true, false, false};
  original |= has_stamp;
  expected = {true, true, false, false};
  EXPECT_EQ(original, expected);

  VertexProperties has_label{false, false, true, false};
  original |= has_label;
  expected = {true, true, true, false};
  EXPECT_EQ(original, expected);

  VertexProperties has_first_seen{false, false, false, true};
  original |= has_first_seen;
  expected = {true, true, true, true};
  EXPECT_EQ(original, expected);
}

}  // namespace kimera_pgmo::traits
