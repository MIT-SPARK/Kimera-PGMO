/**
 * @file   MeshDelta.cpp
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#include "kimera_pgmo/MeshDelta.h"

#include <numeric>

namespace kimera_pgmo {

struct LabeledCloud {
  using Cloud = pcl::PointCloud<pcl::PointXYZRGBA>;

  LabeledCloud(Cloud& points, std::vector<traits::Label>& labels)
      : points(points), labels(labels) {}

  Cloud& points;
  std::vector<traits::Label>& labels;
};

struct LabeledStampedCloud : LabeledCloud {
  using Cloud = pcl::PointCloud<pcl::PointXYZRGBA>;

  LabeledStampedCloud(Cloud& points,
                      std::vector<traits::Timestamp>& stamps,
                      std::vector<traits::Label>& labels)
      : LabeledCloud(points, labels), stamps(stamps) {}

  std::vector<traits::Timestamp>& stamps;
};

size_t pgmoNumVertices(const LabeledCloud& cloud) { return cloud.points.size(); }

void pgmoResizeVertices(LabeledCloud& cloud, size_t size) {
  cloud.points.resize(size);
  cloud.labels.resize(size);
}

size_t pgmoNumVertices(const LabeledStampedCloud& cloud) { return cloud.points.size(); }

void pgmoResizeVertices(LabeledStampedCloud& cloud, size_t size) {
  pgmoResizeVertices(static_cast<LabeledCloud&>(cloud), size);
  cloud.stamps.resize(size);
}

void pgmoSetVertex(LabeledCloud& cloud,
                   size_t i,
                   const traits::Pos& pos,
                   const std::optional<traits::Color>& color,
                   const std::optional<uint8_t>& alpha,
                   const std::optional<traits::Timestamp>& stamp,
                   const std::optional<traits::Label>& label) {
  pgmoSetVertex(cloud.points, i, pos, color, alpha, stamp, label);
  if (label) {
    cloud.labels.at(i) = *label;
  }
}

void pgmoSetVertex(LabeledStampedCloud& cloud,
                   size_t i,
                   const traits::Pos& pos,
                   const std::optional<traits::Color>& color,
                   const std::optional<uint8_t>& alpha,
                   const std::optional<traits::Timestamp>& stamp,
                   const std::optional<traits::Label>& label) {
  pgmoSetVertex(static_cast<LabeledCloud&>(cloud), i, pos, color, alpha, stamp, label);
  if (stamp) {
    cloud.stamps.at(i) = *stamp;
  }
}

std::ostream& operator<<(std::ostream& out, const Face& face) {
  out << "(" << face.v1 << ", " << face.v2 << ", " << face.v3 << ")";
  return out;
}

Face::Face(size_t v1, size_t v2, size_t v3) : v1(v1), v2(v2), v3(v3) {}

Face::Face(const std::vector<size_t>& indices, size_t i)
    : v1(indices.at(i)), v2(indices.at(i + 1)), v3(indices.at(i + 2)) {}

bool Face::valid() const { return v1 != v2 && v1 != v3 && v2 != v3; }

void Face::fill(std::vector<uint32_t>& other) const { other = {v1, v2, v3}; }

MeshDelta::MeshDelta() : MeshDelta(0, 0) {}

MeshDelta::MeshDelta(size_t vertex_start, size_t face_start)
    : vertex_start(vertex_start), face_start(face_start) {
  vertex_updates.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
}

MeshDelta::MeshDelta(const KimeraPgmoMeshDelta& msg)
    : vertex_start(msg.vertex_start),
      face_start(msg.face_start),
      stamp_updates(msg.stamp_updates),
      semantic_updates(msg.semantic_updates) {
  assert(msg.vertex_updates.size() == msg.vertex_updates_colors.size());

  vertex_updates.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
  vertex_updates->resize(msg.vertex_updates.size());
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
    (*vertex_updates)[i] = v;
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

MeshDelta::MeshDelta(const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                     const std::vector<uint64_t>& stamps,
                     const std::vector<pcl::Vertices>& faces,
                     std::optional<std::vector<uint32_t>> semantics) {
  vertex_start = 0;
  face_start = 0;

  vertex_updates.reset(new pcl::PointCloud<pcl::PointXYZRGBA>(vertices));
  stamp_updates = stamps;

  if (semantics) {
    semantic_updates = *semantics;
  }

  face_updates.reserve(faces.size());
  for (const auto& face : faces) {
    Face f(face.vertices[0], face.vertices[1], face.vertices[2]);
    face_updates.push_back(f);
  }
}

bool MeshDelta::hasSemantics() const {
  return semantic_updates.size() == vertex_updates->size();
}

void MeshDelta::updateVertices(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                               std::vector<Timestamp>* stamps,
                               std::vector<uint32_t>* semantics) const {
  const bool semantics_valid = semantics && hasSemantics();
  if (!stamps && !semantics) {
    updateVertices(vertices);
  } else if (!semantics_valid) {
    StampedCloud<pcl::PointXYZRGBA> cloud{vertices, *stamps};
    updateVertices(cloud);
  } else if (!stamps) {
    LabeledCloud cloud{vertices, *semantics};
    updateVertices(cloud);
  } else {
    LabeledStampedCloud cloud{vertices, *stamps, *semantics};
    updateVertices(cloud);
  }
}

void MeshDelta::updateMesh(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                           std::vector<Timestamp>& stamps,
                           std::vector<pcl::Vertices>& faces,
                           std::vector<uint32_t>* semantics) const {
  updateVertices(vertices, &stamps, semantics);
  updateFaces(faces);
}

size_t MeshDelta::addVertex(uint64_t timestamp_ns,
                            const pcl::PointXYZRGBA& point,
                            std::optional<uint32_t> semantics,
                            bool archive) {
  const size_t index = vertex_start + vertex_updates->size();
  vertex_updates->push_back(point);
  stamp_updates.push_back(timestamp_ns);

  if (semantics) {
    semantic_updates.push_back(*semantics);
  }

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

size_t MeshDelta::getLocalIndex(size_t index) const { return index - vertex_start; }

size_t MeshDelta::getGlobalIndex(size_t index) const { return index + vertex_start; }

pcl::IndicesPtr MeshDelta::getActiveIndices() const {
  const size_t num_active = vertex_updates->size() - num_archived_vertices_;
  // hide the face that the underlying indice type changes
  pcl::IndicesPtr indices(new pcl::IndicesPtr::element_type(num_active));
  std::iota(indices->begin(), indices->end(), vertex_start + num_archived_vertices_);
  return indices;
}

void MeshDelta::validate(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                         std::vector<Timestamp>& stamps,
                         std::vector<pcl::Vertices>& faces,
                         std::vector<uint32_t>* semantics) const {
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

  if (semantics && semantics->size() < vertex_start) {
    std::stringstream ss;
    ss << "invalid update: semantics size (" << semantics->size()
       << ") < vertex start (" << vertex_start << ")";
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

  const size_t total_vertices = vertex_start + vertex_updates->size();
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
  const auto& vertices = *vertex_updates;
  mesh_delta_msg.vertex_updates.resize(vertices.size());
  mesh_delta_msg.vertex_updates_colors.resize(vertices.size());
  for (size_t i = 0; i < vertices.size(); i++) {
    geometry_msgs::Point vertex_p;
    vertex_p.x = vertices[i].x;
    vertex_p.y = vertices[i].y;
    vertex_p.z = vertices[i].z;
    mesh_delta_msg.vertex_updates[i] = vertex_p;
    // Point color
    std_msgs::ColorRGBA vertex_c;
    constexpr float color_conv_factor = 1.0f / std::numeric_limits<uint8_t>::max();
    vertex_c.r = color_conv_factor * static_cast<float>(vertices[i].r);
    vertex_c.g = color_conv_factor * static_cast<float>(vertices[i].g);
    vertex_c.b = color_conv_factor * static_cast<float>(vertices[i].b);
    vertex_c.a = color_conv_factor * static_cast<float>(vertices[i].a);
    mesh_delta_msg.vertex_updates_colors[i] = vertex_c;
  }
  mesh_delta_msg.stamp_updates = stamp_updates;
  if (hasSemantics()) {
    mesh_delta_msg.semantic_updates = semantic_updates;
  }
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

std::ostream& operator<<(std::ostream& out, const MeshDelta& delta) {
  out << "vertex start: " << delta.vertex_start << ", face start: " << delta.face_start
      << std::endl;
  out << "points:" << std::endl;
  for (size_t i = 0; i < delta.vertex_updates->size(); ++i) {
    out << "  - " << i << ": " << delta.vertex_updates->at(i) << " @ "
        << delta.stamp_updates.at(i);
    if (delta.hasSemantics()) {
      out << " (label=" << delta.semantic_updates.at(i) << ")";
    }
    out << std::endl;
  }

  out << "faces:" << std::endl;
  for (const auto& face : delta.face_updates) {
    out << " - " << face << std::endl;
  }

  out << "archived faces:" << std::endl;
  for (const auto& face : delta.face_archive_updates) {
    out << " - " << face << std::endl;
  }

  return out;
}

}  // namespace kimera_pgmo
