#pragma once
#include <memory>

#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>
#include <rviz_common/message_filter_display.hpp>

namespace rviz_common::properties {
class BoolProperty;
class ColorProperty;
}  // namespace rviz_common::properties

namespace kimera_pgmo {

class MeshVisual;
class MeshProperties;

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

  void meshSlot();

 private:
  std::unique_ptr<MeshVisual> visual_;
  std::unique_ptr<rviz_common::properties::BoolProperty> visibility_;
  std::unique_ptr<rviz_common::properties::BoolProperty> cull_;
  std::unique_ptr<rviz_common::properties::BoolProperty> lighting_;

  std::unique_ptr<rviz_common::properties::ColorProperty> ambient_;
  std::unique_ptr<rviz_common::properties::ColorProperty> emissive_;
  std::unique_ptr<rviz_common::properties::ColorProperty> diffuse_;
  std::unique_ptr<rviz_common::properties::ColorProperty> specular_;

  std::unique_ptr<MeshProperties> mesh_properties_;
};

}  // namespace kimera_pgmo
