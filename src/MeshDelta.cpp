/**
 * @file   MeshDelta.cpp
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#include "kimera_pgmo/MeshDelta.h"

#include <numeric>

namespace kimera_pgmo {

std::ostream& operator<<(std::ostream& out, const Face& face) {
  out << "(" << face.v1 << ", " << face.v2 << ", " << face.v3 << ")";
  return out;
}

Face::Face(size_t v1, size_t v2, size_t v3) : v1(v1), v2(v2), v3(v3) {}

Face::Face(const std::vector<size_t>& indices, size_t i)
    : v1(indices.at(i)), v2(indices.at(i + 1)), v3(indices.at(i + 2)) {}

bool Face::valid() const { return v1 != v2 && v1 != v3 && v2 != v3; }

void Face::fill(std::vector<uint32_t>& other) const { other = {v1, v2, v3}; }

MeshDelta::MeshDelta(size_t vertex_start, size_t face_start)
    : vertex_start(vertex_start), face_start(face_start) {}

MeshDelta::MeshDelta(const KimeraPgmoMeshDelta& msg)
    : vertex_start(msg.vertex_start),
      face_start(msg.face_start),
      stamp_updates(msg.stamp_updates) {
  assert(msg.vertex_updates.size() == msg.vertex_updates.colors.size());

  vertex_updates.resize(msg.vertex_updates.size());
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
    vertex_updates[i] = v;
  }

  deleted_indices =
      std::set<size_t>(msg.deleted_indices.begin(), msg.deleted_indices.end());

  for (size_t i = 0; i < msg.face_updates.size(); i++) {
    Face face(msg.face_updates[i].vertex_indices[0],
              msg.face_updates[i].vertex_indices[1],
              msg.face_updates[i].vertex_indices[2]);
    face_updates.push_back(face);
  }
  for (size_t i = 0; i < msg.face_archive_updates.size(); i++) {
    Face face(msg.face_archive_updates[i].vertex_indices[0],
              msg.face_archive_updates[i].vertex_indices[1],
              msg.face_archive_updates[i].vertex_indices[2]);
    face_archive_updates.push_back(face);
  }
}

void MeshDelta::updateVertices(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                               std::vector<ros::Time>* stamps) const {
  const size_t total_vertices = vertex_start + vertex_updates.size();
  vertices.resize(total_vertices);
  if (stamps) {
    stamps->resize(total_vertices);
  }

  for (size_t i = 0; i < vertex_updates.size(); ++i) {
    const size_t idx = i + vertex_start;
    vertices[idx] = vertex_updates.at(i);
    if (stamps) {
      stamps->at(idx).fromNSec(stamp_updates.at(i));
    }
  }
}

void MeshDelta::updateMesh(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                           std::vector<ros::Time>& stamps,
                           std::vector<pcl::Vertices>& faces) const {
  updateVertices(vertices, &stamps);

  const size_t total_faces =
      face_start + face_archive_updates.size() + face_updates.size();
  faces.resize(total_faces);

  size_t face_idx = face_start;
  for (const auto& face : face_archive_updates) {
    face.fill(faces[face_idx].vertices);
    ++face_idx;
  }

  for (const auto& face : face_updates) {
    face.fill(faces[face_idx].vertices);
    ++face_idx;
  }
}

size_t MeshDelta::addVertex(uint64_t timestamp_ns,
                            const pcl::PointXYZRGBA& point,
                            bool archive) {
  const size_t index = vertex_start + vertex_updates.size();
  vertex_updates.push_back(point);
  stamp_updates.push_back(timestamp_ns);

  if (archive) {
    num_archived_vertices_++;
  }

  return index;
}

void MeshDelta::addFace(const Face& face, bool archive) {
  if (archive) {
    face_archive_updates.push_back(face);
  } else {
    face_updates.push_back(face);
  }
}

size_t MeshDelta::getNumArchivedVertices() const { return num_archived_vertices_; }

size_t MeshDelta::getNumArchivedFaces() const { return face_archive_updates.size(); }

size_t MeshDelta::getTotalArchivedVertices() const {
  return vertex_start + num_archived_vertices_;
}

size_t MeshDelta::getTotalArchivedFaces() const {
  return face_start + face_archive_updates.size();
}

pcl::IndicesPtr MeshDelta::getActiveIndices() const {
  const size_t num_active = vertex_updates.size() - num_archived_vertices_;
  // hide the face that the underlying indice type changes
  pcl::IndicesPtr indices(new pcl::IndicesPtr::element_type(num_active));
  std::iota(indices->begin(), indices->end(), vertex_start + num_archived_vertices_);
  return indices;
}

void MeshDelta::validate(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                         std::vector<ros::Time>& stamps,
                         std::vector<pcl::Vertices>& faces) const {
  if (vertices.size() < vertex_start) {
    std::stringstream ss;
    ss << "invalid update: vertices size (" << vertices.size() << ") < vertex start ("
       << vertex_start << ")";
    throw std::runtime_error(ss.str());
  }

  if (stamps.size() < vertex_start) {
    std::stringstream ss;
    ss << "invalid update: stamps size (" << stamps.size() << ") < vertex start ("
       << vertex_start << ")";
    throw std::runtime_error(ss.str());
  }

  if (faces.size() < face_start) {
    std::stringstream ss;
    ss << "invalid update: faces size (" << faces.size() << ") < faces start ("
       << face_start << ")";
    throw std::runtime_error(ss.str());
  }

  checkFaces("[validate]");
}

void MeshDelta::checkFaces(const std::string& name) const {
  const size_t total_archived = getTotalArchivedVertices();
  for (const auto& face : face_archive_updates) {
    bool valid_archive = face.v1 < total_archived && face.v2 < total_archived &&
                         face.v3 < total_archived;
    if (!valid_archive) {
      std::stringstream ss;
      ss << name << " invalid archived face: " << face
         << " (total archived: " << total_archived << ")";
      throw std::runtime_error(ss.str());
    }
  }

  const size_t total_vertices = vertex_start + vertex_updates.size();
  for (const auto& face : face_updates) {
    bool valid = face.v1 < total_vertices && face.v2 < total_vertices &&
                 face.v3 < total_vertices;
    if (!valid) {
      std::stringstream ss;
      ss << name << " invalid face: " << face << " (total vertices: " << total_vertices
         << ")";
      throw std::runtime_error(ss.str());
    }
  }
}

KimeraPgmoMeshDelta MeshDelta::toRosMsg(uint64_t timestamp_ns) const {
  KimeraPgmoMeshDelta mesh_delta_msg;

  mesh_delta_msg.header.stamp.fromNSec(timestamp_ns);

  mesh_delta_msg.vertex_start = vertex_start;
  mesh_delta_msg.face_start = face_start;

  // Convert vertices
  mesh_delta_msg.vertex_updates.resize(vertex_updates.size());
  mesh_delta_msg.vertex_updates_colors.resize(vertex_updates.size());
  for (size_t i = 0; i < vertex_updates.size(); i++) {
    geometry_msgs::Point vertex_p;
    vertex_p.x = vertex_updates[i].x;
    vertex_p.y = vertex_updates[i].y;
    vertex_p.z = vertex_updates[i].z;
    mesh_delta_msg.vertex_updates[i] = vertex_p;
    // Point color
    std_msgs::ColorRGBA vertex_c;
    constexpr float color_conv_factor = 1.0f / std::numeric_limits<uint8_t>::max();
    vertex_c.r = color_conv_factor * static_cast<float>(vertex_updates[i].r);
    vertex_c.g = color_conv_factor * static_cast<float>(vertex_updates[i].g);
    vertex_c.b = color_conv_factor * static_cast<float>(vertex_updates[i].b);
    vertex_c.a = color_conv_factor * static_cast<float>(vertex_updates[i].a);
    mesh_delta_msg.vertex_updates_colors[i] = vertex_c;
  }
  mesh_delta_msg.stamp_updates = stamp_updates;
  mesh_delta_msg.deleted_indices.resize(deleted_indices.size());
  std::copy(deleted_indices.begin(),
            deleted_indices.end(),
            mesh_delta_msg.deleted_indices.begin());

  mesh_delta_msg.face_updates.resize(face_updates.size());
  mesh_delta_msg.face_archive_updates.resize(face_archive_updates.size());
  for (size_t i = 0; i < face_updates.size(); i++) {
    TriangleIndices face;
    face.vertex_indices[0] = face_updates[i].v1;
    face.vertex_indices[1] = face_updates[i].v2;
    face.vertex_indices[2] = face_updates[i].v3;
    mesh_delta_msg.face_updates[i] = face;
  }

  for (size_t i = 0; i < face_archive_updates.size(); i++) {
    TriangleIndices face;
    face.vertex_indices[0] = face_archive_updates[i].v1;
    face.vertex_indices[1] = face_archive_updates[i].v2;
    face.vertex_indices[2] = face_archive_updates[i].v3;
    mesh_delta_msg.face_archive_updates[i] = face;
  }

  for (const auto& prev_curr : prev_to_curr) {
    mesh_delta_msg.prev_indices.push_back(prev_curr.first);
    mesh_delta_msg.curr_indices.push_back(prev_curr.second);
  }

  return mesh_delta_msg;
}

}  // namespace kimera_pgmo
