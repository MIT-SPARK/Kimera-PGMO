#pragma once

#include <kimera_pgmo/mesh_types.h>

#include <kimera_pgmo_msgs/msg/triangle_indices.hpp>
#include <kimera_pgmo_msgs/msg/vertex.hpp>

namespace kimera_pgmo::conversions {

void to_ros(const traits::Vertex& vertex, kimera_pgmo_msgs::msg::Vertex& msg);
traits::Vertex from_ros(const kimera_pgmo_msgs::msg::Vertex& msg);

void to_ros(const traits::Face& face, kimera_pgmo_msgs::msg::TriangleIndices& msg);
traits::Face from_ros(const kimera_pgmo_msgs::msg::TriangleIndices& msg);

}  // namespace kimera_pgmo::conversions
