/**
 * @file   MeshDisplay.cpp
 * @brief  Rviz display for viewing mesh
 * @author Nathan Hughes
 */
#include "kimera_pgmo_rviz/MeshDisplay.h"

#include <pluginlib/class_list_macros.h>
#include <rviz/properties/bool_property.h>

#include "kimera_pgmo_rviz/MeshVisual.h"

namespace kimera_pgmo {

using kimera_pgmo::KimeraPgmoMesh;

MeshDisplay::MeshDisplay() {
  cull_ = std::make_unique<rviz::BoolProperty>(
      "Cull Backfaces", true, "Toggle culling backfaces", this, SLOT(updateSettings()));
  lighting_ = std::make_unique<rviz::BoolProperty>("Enable Lighting",
                                                   false,
                                                   "Toggle enabling lighting",
                                                   this,
                                                   SLOT(updateSettings()));
}

MeshDisplay::~MeshDisplay() {}

void MeshDisplay::onInitialize() { MFDClass::onInitialize(); }

void MeshDisplay::reset() {
  MFDClass::reset();
  mesh_.reset();
}

void MeshDisplay::updateSettings() {
  if (mesh_) {
    mesh_->shouldCull(cull_->getBool());
    mesh_->shouldLight(lighting_->getBool());
  }
}

void MeshDisplay::processMessage(const KimeraPgmoMesh::ConstPtr& msg) {
  if (!msg) {
    return;
  }

  Ogre::Vector3 rviz_t_msg;
  Ogre::Quaternion rviz_R_msg;
  if (!context_->getFrameManager()->getTransform(
          msg->header.frame_id, msg->header.stamp, rviz_t_msg, rviz_R_msg)) {
    ROS_DEBUG_STREAM("Failed to transform msg frame "
                     << msg->header.frame_id << " to " << qPrintable(fixed_frame_)
                     << " @ " << msg->header.stamp.toNSec() << " [ns]");
    return;
  }

  if (!mesh_) {
    mesh_ = std::make_unique<MeshVisual>(context_->getSceneManager(), scene_node_);
    mesh_->shouldCull(cull_->getBool());
  }

  mesh_->setMessage(*msg, rviz_t_msg, rviz_R_msg);
}

}  // namespace kimera_pgmo

PLUGINLIB_EXPORT_CLASS(kimera_pgmo::MeshDisplay, rviz::Display)
