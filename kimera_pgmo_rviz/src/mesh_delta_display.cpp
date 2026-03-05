/**
 * @file   mesh_display.cpp
 * @brief  Rviz display for viewing mesh
 * @author Nathan Hughes
 */
#include "kimera_pgmo_rviz/mesh_delta_display.h"

#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo_ros/conversion/mesh_delta.h>

#include <rviz_common/logging.hpp>
#include <rviz_common/properties/bool_property.hpp>

#include "kimera_pgmo_rviz/mesh_visual.h"

namespace kimera_pgmo {

using rviz_common::properties::BoolProperty;

MeshDeltaDisplay::MeshDeltaDisplay() {
  visibility_ = std::make_unique<BoolProperty>(
      "Visible", true, "Toggle visibility", this, SLOT(settingsSlot()));
  cull_ = std::make_unique<BoolProperty>(
      "Cull Backfaces", true, "Toggle backface culling", this, SLOT(settingsSlot()));
  lighting_ = std::make_unique<BoolProperty>(
      "Enable Lighting", false, "Toggle lighting", this, SLOT(settingsSlot()));
}

MeshDeltaDisplay::~MeshDeltaDisplay() {}

void MeshDeltaDisplay::onInitialize() { MFDClass::onInitialize(); }

void MeshDeltaDisplay::reset() {
  MFDClass::reset();

  faces_.clear();
  vertices_.clear();
  visual_.reset();
}

void MeshDeltaDisplay::settingsSlot() {
  if (!visual_) {
    return;
  }

  visual_->setVisible(visibility_->getBool());
  visual_->shouldCull(cull_->getBool());
  visual_->shouldLight(lighting_->getBool());
}

void MeshDeltaDisplay::processMessage(const Msg::ConstSharedPtr msg) {
  if (!msg) {
    return;
  }

  auto delta = conversions::from_ros(*msg);
  delta->updateMesh(vertices_, faces_, offsets_);

  if (!visual_) {
    visual_ = std::make_unique<MeshVisual>(
        context_->getSceneManager(), scene_node_, "mesh_delta_display");
    settingsSlot();
  }

  Ogre::Vector3 position;
  Ogre::Quaternion orientation;
  if (!context_->getFrameManager()->getTransform(msg->header, position, orientation)) {
    QString status = "Error transforming from frame '" +
                     QString(msg->header.frame_id.c_str()) + "' to frame '" +
                     fixed_frame_ + "'";
    setStatus(rviz_common::properties::StatusProperty::Error, "Topic", status);
  }

  visual_->setPose(position, orientation);
  visual_->setMesh(vertices_, faces_);
}

}  // namespace kimera_pgmo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(kimera_pgmo::MeshDeltaDisplay, rviz_common::Display)
