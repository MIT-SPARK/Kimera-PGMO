/**
 * @file   MeshDisplay.h
 * @brief  Rviz display for viewing mesh
 * @author Nathan Hughes
 */
#pragma once
#include <kimera_pgmo/KimeraPgmoMesh.h>
#include <rviz/message_filter_display.h>

#include <memory>

namespace rviz {
class BoolProperty;
}

namespace kimera_pgmo {

class MeshVisual;

class MeshDisplay : public rviz::MessageFilterDisplay<kimera_pgmo::KimeraPgmoMesh> {
  Q_OBJECT
 public:
  MeshDisplay();

  virtual ~MeshDisplay();

 protected:
  void onInitialize() override;

  void reset() override;

 private Q_SLOTS:
  void updateSettings();

 private:
  void processMessage(const kimera_pgmo::KimeraPgmoMesh::ConstPtr& msg) override;

  std::unique_ptr<MeshVisual> mesh_;

  // properties
  std::unique_ptr<rviz::BoolProperty> cull_;
  std::unique_ptr<rviz::BoolProperty> lighting_;
};

}  // namespace kimera_pgmo
