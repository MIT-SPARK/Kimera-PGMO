#include "kimera_pgmo_ros/conversion/mesh_types.h"

namespace kimera_pgmo::conversions {
namespace {

void pointFromPgmo(const kimera_pgmo::traits::Pos& pos,
                   geometry_msgs::msg::Point& msg) {
  msg.x = pos.x();
  msg.y = pos.y();
  msg.z = pos.z();
}

void colorFromPgmo(const kimera_pgmo::traits::Color& color,
                   std_msgs::msg::ColorRGBA& msg) {
  static constexpr float color_conv_factor = 1.0f / std::numeric_limits<uint8_t>::max();
  msg.r = color_conv_factor * static_cast<float>(color[0]);
  msg.g = color_conv_factor * static_cast<float>(color[1]);
  msg.b = color_conv_factor * static_cast<float>(color[2]);
  msg.a = color_conv_factor * static_cast<float>(color[3]);
}

traits::Pos pointFromRos(const geometry_msgs::msg::Point& msg) {
  return traits::Pos(msg.x, msg.y, msg.z);
}

traits::Color colorFromRos(const std_msgs::msg::ColorRGBA& msg) {
  static constexpr float color_conv_factor = std::numeric_limits<uint8_t>::max();
  return {static_cast<uint8_t>(color_conv_factor * msg.r),
          static_cast<uint8_t>(color_conv_factor * msg.g),
          static_cast<uint8_t>(color_conv_factor * msg.b),
          static_cast<uint8_t>(color_conv_factor * msg.a)};
}

}  // namespace

void to_ros(const traits::Vertex& v, kimera_pgmo_msgs::msg::Vertex& msg) {
  pointFromPgmo(v.pos, msg.pos);
  if (v.traits.properties.has_color) {
    msg.has_color = true;
    colorFromPgmo(v.traits.color, msg.color);
  }

  if (v.traits.properties.has_stamp) {  // Point color
    msg.has_stamp = true;
    msg.stamp = v.traits.stamp;
  }

  if (v.traits.properties.has_label) {
    msg.has_label = true;
    msg.label = v.traits.label;
  }

  if (v.traits.properties.has_first_seen_stamp) {
    msg.has_first_seen_stamp = true;
    msg.first_seen_stamp = v.traits.first_seen_stamp;
  }
}

traits::Vertex from_ros(const kimera_pgmo_msgs::msg::Vertex& msg) {
  traits::Vertex v;
  v.pos = pointFromRos(msg.pos);
  v.traits.properties.has_color = msg.has_color;
  v.traits.properties.has_stamp = msg.has_stamp;
  v.traits.properties.has_label = msg.has_label;
  v.traits.properties.has_first_seen_stamp = msg.has_first_seen_stamp;
  if (v.traits.properties.has_color) {
    v.traits.color = colorFromRos(msg.color);
  }

  if (v.traits.properties.has_stamp) {
    v.traits.stamp = msg.stamp;
  }

  if (v.traits.properties.has_label) {
    v.traits.label = msg.label;
  }

  if (v.traits.properties.has_first_seen_stamp) {
    v.traits.first_seen_stamp = msg.first_seen_stamp;
  }

  return v;
}

void to_ros(const traits::Face& face, kimera_pgmo_msgs::msg::TriangleIndices& msg) {
  msg.vertex_indices = {static_cast<uint32_t>(face[0]),
                        static_cast<uint32_t>(face[1]),
                        static_cast<uint32_t>(face[2])};
}

traits::Face from_ros(const kimera_pgmo_msgs::msg::TriangleIndices& msg) {
  return {msg.vertex_indices[0], msg.vertex_indices[1], msg.vertex_indices[2]};
}

}  // namespace kimera_pgmo::conversions
