/**
 * @file   mesh_delta_conversion.cpp
 * @brief  Conversion to/from ROS
 * @author Nathan Hughes
 * @author Yun Chang
 */

#include "kimera_pgmo_ros/conversion/mesh_delta_conversion.h"

#include <numeric>

namespace kimera_pgmo::conversions {

using kimera_pgmo_msgs::KimeraPgmoMeshDelta;

MeshDelta::Ptr fromMsg(const KimeraPgmoMeshDelta& msg) {
  auto delta = std::make_shared<MeshDelta>(msg.vertex_start, msg.face_start);
  delta->stamp_updates = msg.stamp_updates;
  delta->semantic_updates = msg.semantic_updates;

  assert(msg.vertex_updates.size() == msg.vertex_updates_colors.size());

  delta->vertex_updates.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  delta->vertex_updates->resize(msg.vertex_updates.size());
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
    (*delta->vertex_updates)[i] = v;
  }

  delta->deleted_indices =
      std::set<size_t>(msg.deleted_indices.begin(), msg.deleted_indices.end());

  std::transform(msg.prev_indices.begin(),
                 msg.prev_indices.end(),
                 msg.curr_indices.begin(),
                 std::inserter(delta->prev_to_curr, delta->prev_to_curr.end()),
                 [](size_t prev, size_t curr) { return std::make_pair(prev, curr); });

  for (size_t i = 0; i < msg.face_updates.size(); i++) {
    Face face(msg.face_updates[i].vertex_indices[0],
              msg.face_updates[i].vertex_indices[1],
              msg.face_updates[i].vertex_indices[2]);
    delta->face_updates.push_back(face);
  }
  for (size_t i = 0; i < msg.face_archive_updates.size(); i++) {
    Face face(msg.face_archive_updates[i].vertex_indices[0],
              msg.face_archive_updates[i].vertex_indices[1],
              msg.face_archive_updates[i].vertex_indices[2]);
    delta->face_archive_updates.push_back(face);
  }

  return delta;
}

KimeraPgmoMeshDelta::ConstPtr toRosMsg(const MeshDelta& delta, uint64_t timestamp_ns) {
  KimeraPgmoMeshDelta::Ptr msg(new KimeraPgmoMeshDelta());
  msg->header.stamp.fromNSec(timestamp_ns);
  msg->vertex_start = delta.vertex_start;
  msg->face_start = delta.face_start;

  // Convert vertices
  const auto& vertices = *delta.vertex_updates;
  msg->vertex_updates.resize(vertices.size());
  msg->vertex_updates_colors.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); i++) {
    geometry_msgs::Point vertex_p;
    vertex_p.x = vertices[i].x;
    vertex_p.y = vertices[i].y;
    vertex_p.z = vertices[i].z;
    msg->vertex_updates[i] = vertex_p;
    // Point color
    std_msgs::ColorRGBA vertex_c;
    constexpr float color_conv_factor = 1.0f / std::numeric_limits<uint8_t>::max();
    vertex_c.r = color_conv_factor * static_cast<float>(vertices[i].r);
    vertex_c.g = color_conv_factor * static_cast<float>(vertices[i].g);
    vertex_c.b = color_conv_factor * static_cast<float>(vertices[i].b);
    vertex_c.a = color_conv_factor * static_cast<float>(vertices[i].a);
    msg->vertex_updates_colors[i] = vertex_c;
  }

  msg->stamp_updates = delta.stamp_updates;
  if (delta.hasSemantics()) {
    msg->semantic_updates = delta.semantic_updates;
  }

  msg->deleted_indices.resize(delta.deleted_indices.size());
  std::copy(delta.deleted_indices.begin(),
            delta.deleted_indices.end(),
            msg->deleted_indices.begin());

  msg->face_updates.resize(delta.face_updates.size());
  msg->face_archive_updates.resize(delta.face_archive_updates.size());
  for (size_t i = 0; i < delta.face_updates.size(); i++) {
    kimera_pgmo_msgs::TriangleIndices face;
    face.vertex_indices[0] = delta.face_updates[i].v1;
    face.vertex_indices[1] = delta.face_updates[i].v2;
    face.vertex_indices[2] = delta.face_updates[i].v3;
    msg->face_updates[i] = face;
  }

  for (size_t i = 0; i < delta.face_archive_updates.size(); i++) {
    kimera_pgmo_msgs::TriangleIndices face;
    face.vertex_indices[0] = delta.face_archive_updates[i].v1;
    face.vertex_indices[1] = delta.face_archive_updates[i].v2;
    face.vertex_indices[2] = delta.face_archive_updates[i].v3;
    msg->face_archive_updates[i] = face;
  }

  for (const auto& prev_curr : delta.prev_to_curr) {
    msg->prev_indices.push_back(prev_curr.first);
    msg->curr_indices.push_back(prev_curr.second);
  }

  return msg;
}

}  // namespace kimera_pgmo::conversions
