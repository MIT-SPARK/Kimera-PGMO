/**
 * @file   mesh_delta_conversion.cpp
 * @brief  Conversion to/from ROS
 * @author Nathan Hughes
 * @author Yun Chang
 */

#include "kimera_pgmo_ros/conversion/mesh_delta.h"
#include <kimera_pgmo/mesh_types.h>

#include <cstdint>
#include <bitset>

#include <rclcpp/time.hpp>

namespace rclcpp {

using DeltaPgmo = kimera_pgmo::MeshDelta;
using DeltaMsg = kimera_pgmo_msgs::msg::MeshDelta;

uint8_t traitsToFieldStatus(const kimera_pgmo::traits::VertexTraits& traits) {
  std::bitset<4> valid;
  valid[0] = traits.color.has_value();
  valid[1] = traits.stamp.has_value();
  valid[2] = traits.label.has_value();
  valid[3] = traits.first_seen_stamp.has_value();
  // this is safe even though bitset returns a wider value than uint8
  return valid.to_ulong();
}

void TypeAdapter<DeltaPgmo, DeltaMsg>::convert_to_ros_message(const custom_type& delta,
                                                              ros_message_type& msg) {
  msg.header.stamp = rclcpp::Time(delta.timestamp_ns);
  msg.sequence_number = delta.info.sequence_number;
  msg.prev_active_vertices = delta.info.prev_active_vertices;
  msg.prev_active_faces = delta.info.prev_active_faces;

  msg.num_archived_vertices = delta.getNumArchivedVertices();
  for (const auto& [prev, curr] : delta.prev_to_curr()) {
    msg.previous_indices.push_back(prev);
    msg.current_indices.push_back(curr);
  }

  const auto& vertices = *delta.vertex_updates;
  msg.vertex_updates.resize(vertices.size());
  msg.vertex_updates_colors.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); i++) {
    geometry_msgs::msg::Point vertex_p;
    vertex_p.x = vertices[i].x;
    vertex_p.y = vertices[i].y;
    vertex_p.z = vertices[i].z;
    msg.vertex_updates[i] = vertex_p;
    // Point color
    std_msgs::msg::ColorRGBA vertex_c;
    constexpr float color_conv_factor = 1.0f / std::numeric_limits<uint8_t>::max();
    vertex_c.r = color_conv_factor * static_cast<float>(vertices[i].r);
    vertex_c.g = color_conv_factor * static_cast<float>(vertices[i].g);
    vertex_c.b = color_conv_factor * static_cast<float>(vertices[i].b);
    vertex_c.a = color_conv_factor * static_cast<float>(vertices[i].a);
    msg.vertex_updates_colors[i] = vertex_c;
  }

  msg.stamp_updates = delta.stamp_updates;
  if (delta.hasSemantics()) {
    msg.semantic_updates = delta.semantic_updates;
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

void TypeAdapter<DeltaPgmo, DeltaMsg>::convert_to_custom(const ros_message_type& msg,
                                                         custom_type& delta) {
  delta = DeltaPgmo(msg.vertex_start, msg.face_start);
  delta.sequence_number = msg.sequence_number;
  delta.stamp_updates = msg.stamp_updates;
  delta.semantic_updates = msg.semantic_updates;
  delta.timestamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();

  assert(msg.vertex_updates.size() == msg.vertex_updates_colors.size());

  delta.vertex_updates.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  delta.vertex_updates->resize(msg.vertex_updates.size());
  constexpr float color_conv_factor = 1.0f * std::numeric_limits<uint8_t>::max();
  for (size_t i = 0; i < msg.vertex_updates.size(); i++) {
    pcl::PointXYZRGBA v;
    v.x = msg.vertex_updates[i].x;
    v.y = msg.vertex_updates[i].y;
    v.z = msg.vertex_updates[i].z;
    v.r = static_cast<uint8_t>(color_conv_factor * msg.vertex_updates_colors[i].r);
    v.g = static_cast<uint8_t>(color_conv_factor * msg.vertex_updates_colors[i].g);
    v.b = static_cast<uint8_t>(color_conv_factor * msg.vertex_updates_colors[i].b);
    v.a = static_cast<uint8_t>(color_conv_factor * msg.vertex_updates_colors[i].a);
    (*delta.vertex_updates)[i] = v;
  }

  delta.deleted_indices =
      std::set<size_t>(msg.deleted_indices.begin(), msg.deleted_indices.end());

  std::transform(msg.prev_indices.begin(),
                 msg.prev_indices.end(),
                 msg.curr_indices.begin(),
                 std::inserter(delta.prev_to_curr, delta.prev_to_curr.end()),
                 [](size_t prev, size_t curr) { return std::make_pair(prev, curr); });

  for (size_t i = 0; i < msg.face_updates.size(); i++) {
    const auto& triangle = msg.face_updates[i].vertex_indices;
    delta.face_updates.emplace_back(triangle[0], triangle[1], triangle[2]);
  }

  for (size_t i = 0; i < msg.face_archive_updates.size(); i++) {
    const auto& triangle = msg.face_archive_updates[i].vertex_indices;
    delta.face_archive_updates.emplace_back(triangle[0], triangle[1], triangle[2]);
  }
}

}  // namespace rclcpp
