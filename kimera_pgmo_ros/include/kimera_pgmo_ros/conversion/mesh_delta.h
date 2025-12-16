#pragma once

#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>

namespace kimera_pgmo {

class MeshDelta;  // forward declare for simplicity

namespace conversions {
void to_ros(const MeshDelta& delta, kimera_pgmo_msgs::msg::MeshDelta& msg);
std::unique_ptr<MeshDelta> from_ros(const kimera_pgmo_msgs::msg::MeshDelta& msg);
}  // namespace conversions

}  // namespace kimera_pgmo
