#include <gtest/gtest.h>

#include <QCoreApplication>
#include <QTemporaryFile>

#include <rviz_common/config.hpp>
#include <rviz_common/properties/color_property.hpp>

#include "kimera_pgmo_rviz/mesh_coloring_property.h"
#include "kimera_pgmo_rviz/mesh_properties.h"

namespace kimera_pgmo {
namespace {

using rviz_common::properties::Property;

class MeshColoringPropertyTest : public ::testing::Test {
 protected:
  Property* select(const std::string& type) {
    selector.setValue(QString::fromStdString("kimera_pgmo/" + type));
    return selector.subProp(QString::fromStdString("kimera_pgmo/" + type));
  }

  Property root;
  MeshColoringProperty selector{&root};
  std::vector<traits::Vertex> vertices{3};
  std::vector<traits::Face> faces;
  std::string ns = "test";
  MeshColoringView view{vertices, faces, ns};
};

TEST_F(MeshColoringPropertyTest, ModesRetainSettingsAndSaveInactiveEditors) {
  auto uniform = select("Uniform");
  uniform->subProp("Color")->setValue(QColor(10, 20, 30));
  uniform->subProp("Opacity")->setValue(0.5);
  EXPECT_EQ(selector.createColoring()->color(view, 0),
            (traits::Color{10, 20, 30, 128}));

  select("Semantic")->subProp("Label Blend")->setValue(0.25);
  rviz_common::Config config;
  selector.save(config);
  Property other_root;
  MeshColoringProperty restored(&other_root);
  restored.load(config);
  EXPECT_EQ(restored.getStdString(), "kimera_pgmo/Semantic");
  EXPECT_DOUBLE_EQ(restored.subProp("kimera_pgmo/Semantic")
                       ->subProp("Label Blend")
                       ->getValue()
                       .toDouble(),
                   0.25);

  restored.setValue("kimera_pgmo/Uniform");
  EXPECT_EQ(restored.createColoring()->color(view, 0),
            (traits::Color{10, 20, 30, 128}));
  EXPECT_TRUE(restored.subProp("kimera_pgmo/Semantic")->getHidden());
  EXPECT_FALSE(restored.subProp("kimera_pgmo/Uniform")->getHidden());
}

TEST_F(MeshColoringPropertyTest, SplitRestoresNestedSelectedAndInactiveSettings) {
  auto split = select("Split");
  auto child = dynamic_cast<MeshColoringProperty*>(split->subProp("Child Coloring"));
  ASSERT_NE(child, nullptr);
  child->setValue("kimera_pgmo/Uniform");
  child->subProp("kimera_pgmo/Uniform")->subProp("Color")->setValue(QColor(1, 2, 3));
  child->setValue("kimera_pgmo/RGB");
  child->setValue("kimera_pgmo/Uniform");

  rviz_common::Config config;
  selector.save(config);
  Property other_root;
  MeshColoringProperty restored(&other_root);
  restored.load(config);
  EXPECT_EQ(restored.getStdString(), "kimera_pgmo/Split");
  EXPECT_EQ(restored.createColoring()->color(view, 0), (traits::Color{1, 2, 3, 255}));
}

TEST_F(MeshColoringPropertyTest, ProcessorsHaveIndependentBoundsAndOutliveEditors) {
  select("LastSeen");
  auto first = selector.createColoring();
  auto second = selector.createColoring();
  for (size_t i = 0; i < vertices.size(); ++i) {
    vertices[i].traits.properties.has_stamp = true;
    vertices[i].traits.stamp = 10 * (i + 1);
  }

  first->prepare(view, 0);
  const auto color = first->color(view, 1);
  vertices[2].traits.stamp = 100;
  second->prepare(view, 0);
  EXPECT_EQ(first->color(view, 1), color);
  EXPECT_NE(second->color(view, 1), color);

  std::shared_ptr<MeshColoring> retained;
  {
    Property temporary_root;
    MeshColoringProperty temporary(&temporary_root);
    temporary.setValue("kimera_pgmo/Uniform");
    temporary.subProp("kimera_pgmo/Uniform")
        ->subProp("Color")
        ->setValue(QColor(4, 5, 6));
    retained = temporary.createColoring();
  }

  EXPECT_EQ(retained->color(view, 0), (traits::Color{4, 5, 6, 255}));
}

TEST_F(MeshColoringPropertyTest,
       TimeFieldsPreserveNanosecondPrecisionAndValidateBounds) {
  auto time = select("FirstSeen");
  EXPECT_TRUE(time->subProp("Minimum Time (ns)")->getHidden());
  time->subProp("Range")->setValue("Fixed");
  EXPECT_FALSE(time->subProp("Minimum Time (ns)")->getHidden());
  time->subProp("Minimum Time (ns)")->setValue("1700000000000000001");
  time->subProp("Maximum Time (ns)")->setValue("1700000000000000003");
  auto processor = selector.createColoring();
  vertices[0].traits.properties.has_first_seen_stamp = true;
  vertices[0].traits.first_seen_stamp = 1700000000000000001ULL;
  EXPECT_EQ(processor->color(view, 0), (traits::Color{0, 0, 0, 255}));
  vertices[0].traits.first_seen_stamp += 2;
  EXPECT_EQ(processor->color(view, 0), (traits::Color{255, 255, 255, 255}));

  time->subProp("Maximum Time (ns)")->setValue("1");
  EXPECT_THROW(selector.createColoring(), std::invalid_argument);
  time->subProp("Minimum Time (ns)")->setValue("-1");
  EXPECT_THROW(selector.createColoring(), std::invalid_argument);
}

TEST_F(MeshColoringPropertyTest, SemanticPaletteFileAndInvalidPlugin) {
  QTemporaryFile file;
  ASSERT_TRUE(file.open());
  file.write("5: [255, 0, 0, 128]\n");
  file.flush();
  auto semantic = select("Semantic");
  semantic->subProp("Palette")->setValue("File");
  semantic->subProp("Palette File")->setValue(file.fileName());
  vertices[0].traits.properties.has_label = true;
  vertices[0].traits.label = 5;
  EXPECT_EQ(selector.createColoring()->color(view, 0), (traits::Color{255, 0, 0, 128}));

  semantic->subProp("Palette File")->setValue("/no/such/palette.yaml");
  EXPECT_THROW(selector.createColoring(), std::exception);
  EXPECT_NO_THROW(selector.setValue("missing/Coloring"));
  EXPECT_THROW(selector.createColoring(), std::exception);
}

TEST(MeshProperties, CoalescesEditsAndKeepsChunkChangesSeparate) {
  Property root;
  MeshProperties properties(&root);
  auto selector = dynamic_cast<MeshColoringProperty*>(root.subProp("Coloring"));
  ASSERT_NE(selector, nullptr);
  size_t changes = 0;
  QObject::connect(
      &properties, &MeshProperties::changed, &root, [&changes]() { ++changes; });
  selector->setValue("kimera_pgmo/Uniform");
  selector->subProp("kimera_pgmo/Uniform")->subProp("Color")->setValue(QColor(1, 2, 3));
  selector->subProp("kimera_pgmo/Uniform")->subProp("Opacity")->setValue(0.5);
  EXPECT_EQ(changes, 0u);
  QCoreApplication::processEvents();
  EXPECT_EQ(changes, 1u);

  const auto revision = properties.coloringRevision();
  root.subProp("Max Chunk Vertices")->setValue(100);
  QCoreApplication::processEvents();
  EXPECT_EQ(changes, 2u);
  EXPECT_EQ(properties.coloringRevision(), revision);
}

}  // namespace
}  // namespace kimera_pgmo
