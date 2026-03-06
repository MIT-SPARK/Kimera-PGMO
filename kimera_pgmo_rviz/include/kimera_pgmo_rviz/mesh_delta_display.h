#pragma once
#include <kimera_pgmo/mesh_offset_info.h>
#include <kimera_pgmo/mesh_types.h>

#include <memory>

#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>
#include <rviz_common/message_filter_display.hpp>

#include "kimera_pgmo_rviz/visuals_map.h"

namespace rviz_common::properties {
class BoolProperty;
class ColorProperty;
class FloatProperty;
}

namespace kimera_pgmo {

class MeshVisual;

class MeshDeltaDisplay
    : public rviz_common::MessageFilterDisplay<kimera_pgmo_msgs::msg::MeshDelta> {
  Q_OBJECT
 public:
  using Msg = kimera_pgmo_msgs::msg::MeshDelta;

  MeshDeltaDisplay();

  ~MeshDeltaDisplay() override;

 protected:
  void onInitialize() override;

  void reset() override;

  void processMessage(const Msg::ConstSharedPtr msg) override;

 private Q_SLOTS:
  void settingsSlot();

  void colorSlot();

 private:
  MeshOffsetInfo offsets_;
  std::vector<traits::Face> faces_;
  std::vector<traits::Vertex> vertices_;

  std::unique_ptr<MeshVisual> visual_;
  std::unique_ptr<rviz_common::properties::BoolProperty> visibility_;
  std::unique_ptr<rviz_common::properties::BoolProperty> cull_;
  std::unique_ptr<rviz_common::properties::BoolProperty> lighting_;

  std::unique_ptr<rviz_common::properties::ColorProperty> ambient_;
  std::unique_ptr<rviz_common::properties::ColorProperty> emissive_;
  std::unique_ptr<rviz_common::properties::ColorProperty> diffuse_;
  std::unique_ptr<rviz_common::properties::ColorProperty> specular_;

  std::unique_ptr<rviz_common::properties::FloatProperty> label_alpha_;
  std::unique_ptr<rviz_common::properties::ColorProperty> default_color_;
};

}  // namespace kimera_pgmo
