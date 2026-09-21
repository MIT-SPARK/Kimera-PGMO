#include <gtest/gtest.h>
#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo_ros/conversion/mesh_delta.h>
#include <kimera_pgmo_ros/mesh_coloring_factories.h>

namespace kimera_pgmo {
namespace {

class MeshColoringTest : public ::testing::Test {
 protected:
  std::vector<traits::Vertex> vertices{3};
  std::vector<traits::Face> faces;
  std::string ns = "test";
  MeshColoringView view{vertices, faces, ns};
};

TEST_F(MeshColoringTest, SourceColorsAndMissingMetadata) {
  auto rgb = makeRgbColoring();
  EXPECT_EQ(rgb->color(view, 0), (traits::Color{102, 102, 102, 255}));
  vertices[0].traits.properties.has_color = true;
  vertices[0].traits.color = {10, 20, 30, 40};
  EXPECT_EQ(rgb->color(view, 0), vertices[0].traits.color);

  SemanticColoringConfig config;
  config.palette = MeshPalette{{5, {255, 0, 0, 255}}};
  auto semantic = makeSemanticColoring(config);
  EXPECT_EQ(semantic->color(view, 0), vertices[0].traits.color);
  vertices[1].traits.properties.has_label = true;
  vertices[1].traits.label = 5;
  EXPECT_EQ(semantic->color(view, 1), (traits::Color{255, 0, 0, 255}));

  auto uniform = makeUniformColoring({{1, 2, 3, 4}});
  EXPECT_EQ(uniform->color(view, 0), (traits::Color{1, 2, 3, 4}));
  EXPECT_EQ(vertices[0].traits.color, (traits::Color{10, 20, 30, 40}));
  EXPECT_FALSE(rgb->prepare(view, 0));
  EXPECT_FALSE(semantic->prepare(view, 0));
}

TEST_F(MeshColoringTest, TimeBoundsInvalidateArchivedColorsAndHandleRemoval) {
  auto coloring = makeLastSeenColoring();
  for (size_t i = 0; i < vertices.size(); ++i) {
    vertices[i].traits.properties.has_stamp = true;
    vertices[i].traits.stamp = 10 * (i + 1);
  }

  EXPECT_TRUE(coloring->prepare(view, 0));
  const auto before = coloring->color(view, 1);
  EXPECT_FALSE(coloring->prepare(view, 2));
  vertices[2].traits.stamp = 50;
  EXPECT_TRUE(coloring->prepare(view, 2));
  EXPECT_NE(before, coloring->color(view, 1));

  vertices.resize(2);
  EXPECT_TRUE(coloring->prepare(view, 2));
  EXPECT_EQ(coloring->color(view, 1), (traits::Color{255, 255, 255, 255}));

  vertices.clear();
  EXPECT_TRUE(coloring->prepare(view, 0));
}

TEST_F(MeshColoringTest, FixedTimeBoundsAndInvalidDurations) {
  SeenDurationColoringConfig config;
  config.bounds = MeshTimeBounds{0, 100};
  auto coloring = makeSeenDurationColoring(config);
  auto& t = vertices[0].traits;
  t.properties.has_stamp = true;
  t.properties.has_first_seen_stamp = true;
  t.first_seen_stamp = 20;
  t.stamp = 10;
  EXPECT_FALSE(coloring->prepare(view, 0));
  EXPECT_EQ(coloring->color(view, 0), (traits::Color{0, 255, 0, 255}));

  t.stamp = 20;
  EXPECT_EQ(coloring->color(view, 0), (traits::Color{0, 0, 0, 255}));

  t.stamp = 120;
  EXPECT_EQ(coloring->color(view, 0), (traits::Color{255, 255, 255, 255}));
  FirstSeenColoringConfig first_config;
  first_config.bounds = MeshTimeBounds{20, 20};
  auto first = makeFirstSeenColoring(first_config);
  EXPECT_EQ(first->color(view, 0), (traits::Color{0, 0, 0, 255}));
}

TEST_F(MeshColoringTest, SplitComposesProcessorsAndPreservesSource) {
  SplitColoringConfig config;
  config.normal = Eigen::Vector3f::UnitX();
  auto coloring = makeSplitColoring(config, makeUniformColoring({{1, 2, 3, 255}}));
  coloring->prepare(view, 0);
  vertices[0].pos.x() = 1.0f;
  vertices[1].pos.x() = -1.0f;
  EXPECT_EQ(coloring->color(view, 0), (traits::Color{1, 2, 3, 255}));
  EXPECT_EQ(coloring->color(view, 1), (traits::Color{102, 102, 102, 255}));
}

TEST_F(MeshColoringTest, InvalidConfiguration) {
  SemanticColoringConfig semantic;
  semantic.alpha = 2.0;
  EXPECT_THROW(makeSemanticColoring(semantic), std::invalid_argument);

  LastSeenColoringConfig last;
  last.bounds = MeshTimeBounds{20, 10};
  EXPECT_THROW(makeLastSeenColoring(last), std::invalid_argument);

  SplitColoringConfig split;
  split.normal.setZero();
  EXPECT_THROW(makeSplitColoring(split, makeRgbColoring()), std::invalid_argument);
  EXPECT_THROW(makeSplitColoring({}, nullptr), std::invalid_argument);
}

TEST(MeshDeltaConversion, RejectMalformedArrays) {
  kimera_pgmo_msgs::msg::MeshDelta msg;
  msg.previous_indices.push_back(0);
  EXPECT_THROW(conversions::from_ros(msg), std::invalid_argument);

  msg.previous_indices.clear();
  msg.num_archived_vertices = 1;
  EXPECT_THROW(conversions::from_ros(msg), std::invalid_argument);
}

}  // namespace
}  // namespace kimera_pgmo
