/**
 * @file   mesh_display.cpp
 * @brief  Rviz display for viewing mesh
 * @author Nathan Hughes
 */
#include "kimera_pgmo_rviz/mesh_display.h"

#include <pluginlib/class_list_macros.h>
#include <rviz/properties/bool_property.h>

#include "kimera_pgmo_rviz/mesh_visual.h"
#include "kimera_pgmo_rviz/tf_event_buffer.h"
#include "kimera_pgmo_rviz/visibility_field.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::KimeraPgmoMesh;

MeshDisplay::MeshDisplay() {
  // Setup rviz properties.
  cull_ = std::make_unique<rviz::BoolProperty>("Cull Backfaces",
                                               true,
                                               "Toggle culling backfaces",
                                               this,
                                               SLOT(updateGlobalSettingsSlot()));
  lighting_ = std::make_unique<rviz::BoolProperty>("Enable Lighting",
                                                   false,
                                                   "Toggle enabling lighting",
                                                   this,
                                                   SLOT(updateGlobalSettingsSlot()));

  visibility_fields_ = std::make_unique<VisibilityField>("Visible", this, this);
  toggle_visibility_all_property_ =
      std::make_unique<rviz::BoolProperty>("Toggle Visibility All",
                                           true,
                                           "Toggle visibility for all meshes",
                                           this,
                                           SLOT(toggleVisibilityAllSloT()));
}

MeshDisplay::~MeshDisplay() {}

void MeshDisplay::onInitialize() {
  tf_buffer_ = std::make_unique<TfEventBuffer>(context_->getFrameManager(),
                                               TfEventBuffer::Config());
  MFDClass::onInitialize();
}

void MeshDisplay::reset() {
  MFDClass::reset();
  visuals_.reset();
  tf_buffer_->reset();
}

void MeshDisplay::update(float /* wall_dt */, float /* ros_dt */) {
  // Update the poses of all visuals if new transforms are available.
  for (const auto& update : tf_buffer_->getTransformUpdates()) {
    auto visual = visuals_.get(update.ns);
    if (visual) {
      visual->setPose(update.position, update.orientation);
    }
  }
}

void MeshDisplay::updateGlobalSettingsSlot() {
  visuals_.applyToAll([this](MeshVisual& visual) { updateVisualSettings(visual); });
}

void MeshDisplay::updateVisualSettings(MeshVisual& visual) const {
  visual.shouldCull(cull_->getBool());
  visual.shouldLight(lighting_->getBool());
}

void MeshDisplay::visibleSlot() { updateVisible(); }

void MeshDisplay::updateVisible() {
  // Set visibility of all visuals base on the configured visbility.
  for (auto& [ns, visual] : visuals_) {
    bool visible = false;
    if (isEnabled()) {
      visible = visibility_fields_->isEnabled(ns);
    }
    visual->setVisible(visible);
  }
}

void MeshDisplay::toggleVisibilityAllSloT() {
  // Toggle all visibility fields except for the root.
  const bool root_visible = visibility_fields_->getBool();
  visibility_fields_->setEnabledForAll(toggle_visibility_all_property_->getBool());
  visibility_fields_->setBool(root_visible);
  updateVisible();
}

void MeshDisplay::processMessage(const KimeraPgmoMesh::ConstPtr& msg) {
  if (!msg) {
    return;
  }
  const std::string& ns = msg->ns;

  if (msg->vertices.empty()) {
    deleteVisual(ns);
    return;
  }

  // Always reset the target frame in case it has changed.
  // TODO(lschmid): Can consider moving this into createVisual if we think the frame
  // names will not change.
  tf_buffer_->addTransformQuery(ns, msg->header.frame_id);

  MeshVisual* visual = visuals_.get(ns);
  if (!visual) {
    visual = createVisual(ns);
    const auto pose = tf_buffer_->getTransform(ns, true);
    if (pose) {
      visual->setPose(pose->position, pose->orientation);
    }
  }

  visual->setMessage(*msg);
}

MeshVisual* MeshDisplay::createVisual(const std::string& ns) {
  auto& visual = visuals_.create(ns, context_->getSceneManager(), scene_node_);
  updateVisualSettings(visual);
  visibility_fields_->addField(ns);
  visual.setVisible(visibility_fields_->isEnabled(ns));
  return &visual;
}

void MeshDisplay::deleteVisual(const std::string& ns) {
  visuals_.deleteVisual(ns);
  visibility_fields_->removeField(ns);
  tf_buffer_->removeTransformQuery(ns);
}

}  // namespace kimera_pgmo

PLUGINLIB_EXPORT_CLASS(kimera_pgmo::MeshDisplay, rviz::Display)
