#include <gtest/gtest.h>

#include <pluginlib/class_loader.hpp>
#include <rviz_common/display.hpp>
#include <rviz_common/properties/property.hpp>

namespace kimera_pgmo {

TEST(MeshColoringLoading, LoadsAllModesAfterDisplayLibrary) {
  pluginlib::ClassLoader<rviz_common::Display> displays("rviz_common",
                                                        "rviz_common::Display");
  for (const auto& name : {"MeshDisplay", "MeshDeltaDisplay"}) {
    SCOPED_TRACE(name);
    const auto display =
        displays.createSharedInstance(std::string("kimera_pgmo_rviz/") + name);
    auto coloring = display->subProp("Coloring");
    for (const auto& mode : {"RGB",
                             "Uniform",
                             "Semantic",
                             "FirstSeen",
                             "LastSeen",
                             "SeenDuration",
                             "Split"}) {
      const auto type = QString("kimera_pgmo/") + mode;
      SCOPED_TRACE(type.toStdString());
      coloring->setValue(type);
      EXPECT_GT(coloring->subProp(type)->numChildren(), 0);
    }
  }
}

}  // namespace kimera_pgmo
