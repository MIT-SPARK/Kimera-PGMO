/**
 * @file   mesh_delta_conversion.cpp
 * @brief  Conversion to/from ROS
 * @author Nathan Hughes
 * @author Yun Chang
 */

#include "kimera_pgmo_ros/conversion/mesh_delta.h"

#include <kimera_pgmo/mesh_delta.h>
#include <kimera_pgmo/mesh_types.h>

#include <cstdint>
#include <memory>

#include <rclcpp/time.hpp>
#include <std_msgs/msg/color_rgba.hpp>

#include "kimera_pgmo_msgs/msg/mesh_delta.hpp"
#include "std_msgs/msg/color_rgba.hpp"

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

void mesh_delta::to_ros(const MeshDelta& delta, kimera_pgmo_msgs::msg::MeshDelta& msg) {
  msg.header.stamp = rclcpp::Time(delta.timestamp_ns);
  msg.seq_number = delta.info.sequence_number;
  msg.prev_active_vertices = delta.info.prev_active_vertices;
  msg.prev_active_faces = delta.info.prev_active_faces;

  msg.num_archived_vertices = delta.getNumArchivedVertices();
  for (const auto& [prev, curr] : delta.prev_to_curr()) {
    msg.previous_indices.push_back(prev);
    msg.current_indices.push_back(curr);
  }

  const auto props = delta.vertex_properties();
  const auto num_vertices = delta.getNumVertices();
  msg.vertex_updates.resize(num_vertices);
  if (props.has_color) {
    msg.color_updates.resize(num_vertices);
  }

  if (props.has_stamp) {
    msg.stamp_updates.resize(num_vertices);
  }

  if (props.has_label) {
    msg.label_updates.resize(num_vertices);
  }

  if (props.has_first_seen_stamp) {
    msg.first_seen_stamp_updates.resize(num_vertices);
  }

  for (size_t i = 0; i < num_vertices; i++) {
    const auto& p = delta.getVertex(i);
    pointFromPgmo(p.pos, msg.vertex_updates[i]);
    if (props.has_color) {
      colorFromPgmo(p.traits.color, msg.color_updates[i]);
    }

    if (props.has_stamp) {  // Point color
      msg.stamp_updates[i] = p.traits.stamp;
    }

    if (props.has_label) {
      msg.label_updates[i] = p.traits.label;
    }

    if (props.has_first_seen_stamp) {
      msg.first_seen_stamp_updates[i] = p.traits.first_seen_stamp;
    }
  }

  msg.face_updates.reserve(delta.face_updates().size());
  for (const auto& delta_face : delta.face_updates()) {
    auto& face = msg.face_updates.emplace_back();
    face.vertex_indices[0] = delta_face[0];
    face.vertex_indices[1] = delta_face[1];
    face.vertex_indices[2] = delta_face[2];
  }

  msg.face_archive_updates.reserve(delta.face_archive_updates().size());
  for (const auto& delta_face : delta.face_archive_updates()) {
    auto& face = msg.face_archive_updates.emplace_back();
    face.vertex_indices[0] = delta_face[0];
    face.vertex_indices[1] = delta_face[1];
    face.vertex_indices[2] = delta_face[2];
  }
}

MeshDelta::Ptr mesh_delta::from_ros(const kimera_pgmo_msgs::msg::MeshDelta& msg) {
  auto delta = std::make_unique<MeshDelta>(MeshDelta::TrackingInfo{
      msg.seq_number, msg.prev_active_vertices, msg.prev_active_faces});
  delta->timestamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();

  traits::VertexProperties props;
  props.has_color = msg.vertex_updates.size() == msg.color_updates.size();
  props.has_stamp = msg.vertex_updates.size() == msg.stamp_updates.size();
  props.has_label = msg.vertex_updates.size() == msg.label_updates.size();
  props.has_first_seen_stamp =
      msg.vertex_updates.size() == msg.first_seen_stamp_updates.size();

  auto& prev_to_curr = delta->prev_to_curr();
  std::transform(msg.previous_indices.begin(),
                 msg.previous_indices.end(),
                 msg.current_indices.begin(),
                 std::inserter(prev_to_curr, prev_to_curr.end()),
                 [](size_t prev, size_t curr) { return std::make_pair(prev, curr); });

  for (size_t i = 0; i < msg.vertex_updates.size(); i++) {
    traits::Pos pos = pointFromRos(msg.vertex_updates[i]);
    traits::VertexTraits traits;
    traits.properties = props;
    if (traits.properties.has_color) {
      traits.color = colorFromRos(msg.color_updates[i]);
    }
    if (traits.properties.has_stamp) {
      traits.stamp = msg.stamp_updates[i];
    }
    if (traits.properties.has_label) {
      traits.label = msg.label_updates[i];
    }
    if (traits.properties.has_first_seen_stamp) {
      traits.first_seen_stamp = msg.first_seen_stamp_updates[i];
    }

    delta->addVertex(pos, traits, i < msg.num_archived_vertices);
  }

  for (size_t i = 0; i < msg.face_updates.size(); i++) {
    const auto& triangle = msg.face_updates[i].vertex_indices;
    delta->addFace({triangle[0], triangle[1], triangle[2]});
  }

  for (size_t i = 0; i < msg.face_archive_updates.size(); i++) {
    const auto& triangle = msg.face_archive_updates[i].vertex_indices;
    delta->addFace({triangle[0], triangle[1], triangle[2]}, true);
  }

  return delta;
}

}  // namespace kimera_pgmo::conversions
