/**
 * @file   mesh_delta.cpp
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#include "kimera_pgmo/mesh_delta.h"

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
                   const traits::VertexTraits& traits) {
  pgmoSetVertex(cloud.points, i, pos, traits);
  if (traits.label) {
    cloud.labels.at(i) = *traits.label;
  }
}

void pgmoSetVertex(LabeledStampedCloud& cloud,
                   size_t i,
                   const traits::Pos& pos,
                   const traits::VertexTraits& traits) {
  pgmoSetVertex(static_cast<LabeledCloud&>(cloud), i, pos, traits);
  if (traits.stamp) {
    cloud.stamps.at(i) = *traits.stamp;
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
