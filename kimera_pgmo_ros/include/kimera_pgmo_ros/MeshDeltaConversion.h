/**
 * @file   MeshDeltaConversion.h
 * @brief  Conversion to/from ROS
 * @author Nathan Hughes
 * @author Yun Chang
 */

#pragma once

#include <kimera_pgmo/MeshDelta.h>
#include <kimera_pgmo_msgs/KimeraPgmoMeshDelta.h>

namespace kimera_pgmo {

MeshDelta::Ptr fromMsg(const kimera_pgmo_msgs::KimeraPgmoMeshDelta& msg);

kimera_pgmo_msgs::KimeraPgmoMeshDelta::ConstPtr toRosMsg(const MeshDelta& delta,
                                                         Timestamp timestamp_ns);

}  // namespace kimera_pgmo
