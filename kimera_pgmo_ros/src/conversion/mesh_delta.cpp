/**
 * @file   mesh_delta_conversion.cpp
 * @brief  Conversion to/from ROS
 * @author Nathan Hughes
 * @author Yun Chang
 */

#include "kimera_pgmo_ros/conversion/mesh_delta.h"

#include <kimera_pgmo/mesh_delta.h>

#include <rclcpp/time.hpp>

#include "kimera_pgmo_ros/conversion/mesh_types.h"

namespace kimera_pgmo::conversions {

void to_ros(const MeshDelta& delta, kimera_pgmo_msgs::msg::MeshDelta& msg) {
  msg.header.stamp = rclcpp::Time(delta.timestamp_ns);
  msg.seq_number = delta.info.sequence_number;
  msg.prev_active_vertices = delta.info.prev_active_vertices;
  msg.prev_active_faces = delta.info.prev_active_faces;

  msg.num_archived_vertices = delta.getNumArchivedVertices();
  if (delta.info.prev_to_curr) {
    for (const auto& [prev, curr] : *delta.info.prev_to_curr) {
      msg.previous_indices.push_back(prev);
      msg.current_indices.push_back(curr);
    }
  }

  // TODO(nathan) think about embedding properties here
  const auto num_vertices = delta.getNumVertices();
  msg.vertex_updates.resize(num_vertices);
  for (size_t i = 0; i < num_vertices; i++) {
    to_ros(delta.getVertex(i), msg.vertex_updates[i]);
  }

  msg.face_updates.reserve(delta.faces().size());
  for (const auto& delta_face : delta.faces()) {
    auto& face = msg.face_updates.emplace_back();
    to_ros(delta_face, face);
  }

  msg.face_archive_updates.reserve(delta.archived_faces().size());
  for (const auto& delta_face : delta.archived_faces()) {
    auto& face = msg.face_archive_updates.emplace_back();
    to_ros(delta_face, face);
  }
}

MeshDelta::Ptr from_ros(const kimera_pgmo_msgs::msg::MeshDelta& msg) {
  auto info = MeshDelta::TrackingInfo::with_remap(
      msg.seq_number, msg.prev_active_vertices, msg.prev_active_faces);
  std::transform(msg.previous_indices.begin(),
                 msg.previous_indices.end(),
                 msg.current_indices.begin(),
                 std::inserter(*info.prev_to_curr, info.prev_to_curr->end()),
                 [](size_t prev, size_t curr) { return std::make_pair(prev, curr); });

  auto delta = std::make_unique<MeshDelta>(info);
  delta->timestamp_ns = rclcpp::Time(msg.header.stamp).nanoseconds();
  for (size_t i = 0; i < msg.vertex_updates.size(); i++) {
    const auto v = from_ros(msg.vertex_updates[i]);
    delta->addVertex(v.pos, v.traits, i < msg.num_archived_vertices);
  }

  for (size_t i = 0; i < msg.face_updates.size(); i++) {
    delta->addFace(from_ros(msg.face_updates[i]));
  }

  for (size_t i = 0; i < msg.face_archive_updates.size(); i++) {
    delta->addFace(from_ros(msg.face_archive_updates[i]), true);
  }

  return delta;
}

}  // namespace kimera_pgmo::conversions
