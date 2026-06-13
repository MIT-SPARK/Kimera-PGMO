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
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/float_property.hpp>

#include "kimera_pgmo_rviz/mesh_visual.h"

namespace kimera_pgmo {

using rviz_common::properties::BoolProperty;
using rviz_common::properties::ColorProperty;
using rviz_common::properties::FloatProperty;

MeshDeltaDisplay::MeshDeltaDisplay() {
  visibility_ = std::make_unique<BoolProperty>(
      "Visible", true, "Toggle visibility", this, SLOT(settingsSlot()));
  cull_ = std::make_unique<BoolProperty>(
      "Cull Backfaces", true, "Toggle backface culling", this, SLOT(settingsSlot()));
  lighting_ = std::make_unique<BoolProperty>(
      "Enable Lighting", false, "Toggle lighting", this, SLOT(settingsSlot()));

  label_alpha_ = std::make_unique<FloatProperty>("Label Alpha",
                                                 0.0,
                                                 "Amount to blend label colors in by",
                                                 this,
                                                 SLOT(colorSlot()));
  label_alpha_->setMin(0.0);
  label_alpha_->setMax(1.0);
  default_color_ =
      std::make_unique<ColorProperty>("Default Color",
                                      QColor::fromRgbF(0.4, 0.4, 0.4),
                                      "Default color for labels outside of range",
                                      this,
                                      SLOT(colorSlot()));

  ambient_ = std::make_unique<ColorProperty>("Ambient",
                                             QColor::fromRgbF(0.9, 0.9, 0.9),
                                             "Ambient lighting parameters",
                                             this,
                                             SLOT(settingsSlot()));
  emissive_ = std::make_unique<ColorProperty>("Emissive",
                                              QColor::fromRgbF(0.1, 0.1, 0.1),
                                              "Emissive lighting parameters",
                                              this,
                                              SLOT(settingsSlot()));
  diffuse_ = std::make_unique<ColorProperty>("Diffuse",
                                             QColor::fromRgbF(0.05, 0.05, 0.05),
                                             "Diffuse lighting parameter",
                                             this,
                                             SLOT(settingsSlot()));
  specular_ = std::make_unique<ColorProperty>("Specular",
                                              QColor::fromRgbF(0.0, 0.0, 0.0),
                                              "Specular lighting parameter",
                                              this,
                                              SLOT(settingsSlot()));
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

  visual_->setLighting(ambient_->getOgreColor(),
                       emissive_->getOgreColor(),
                       diffuse_->getOgreColor(),
                       specular_->getOgreColor());
}

void MeshDeltaDisplay::colorSlot() {
  if (!visual_) {
    return;
  }

  visual_->setMesh(
      vertices_, faces_, label_alpha_->getFloat(), default_color_->getOgreColor());
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
  visual_->setMesh(
      vertices_, faces_, label_alpha_->getFloat(), default_color_->getOgreColor());
}

}  // namespace kimera_pgmo

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(kimera_pgmo::MeshDeltaDisplay, rviz_common::Display)
