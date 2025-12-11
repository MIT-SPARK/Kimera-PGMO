#pragma once

#include <kimera_pgmo/mesh_delta.h>

#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>

namespace kimera_pgmo::conversions {

void to_ros(const MeshDelta& delta, kimera_pgmo_msgs::msg::MeshDelta& msg);
MeshDelta::Ptr from_ros(const kimera_pgmo_msgs::msg::MeshDelta& msg);

}  // namespace kimera_pgmo::conversions
