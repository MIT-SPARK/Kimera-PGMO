/**
 * @file   mesh_delta.h
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#pragma once

#include <pcl/Vertices.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <optional>
#include <set>
#include <vector>

#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo/pcl_mesh_traits.h"
#include "kimera_pgmo/utils/common_structs.h"

namespace kimera_pgmo {

struct Face {
  Face(size_t v1, size_t v2, size_t v3);

  Face(const std::vector<size_t>& indices, size_t i);
  Face(const traits::Face& face);

  bool valid() const;

  void fill(std::vector<uint32_t>& other) const;

  uint32_t v1;
  uint32_t v2;
  uint32_t v3;

  operator std::array<size_t, 3>() const { return {v1, v2, v3}; }
};

std::ostream& operator<<(std::ostream& out, const Face& face);

class MeshDelta {
 public:
  using Ptr = std::shared_ptr<MeshDelta>;

  MeshDelta();

  MeshDelta(size_t vertex_start, size_t face_start);

  MeshDelta(const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
            const std::vector<Timestamp>& stamps,
            const std::vector<pcl::Vertices>& faces,
            std::optional<std::vector<uint32_t>> semantics = std::nullopt);

  template <typename Vertices, typename Faces>
  static MeshDelta::Ptr fromMesh(const Vertices& vertices, const Faces& faces);

  template <typename Mesh>
  static MeshDelta::Ptr fromMesh(const Mesh& mesh);

  template <typename Vertices>
  void updateVertices(Vertices& vertices, const Eigen::Isometry3f* = nullptr) const;

  template <typename Faces>
  void updateFaces(Faces& faces) const;

  template <typename Vertices, typename Faces>
  void updateMesh(Vertices& vertices,
                  Faces& faces,
                  const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Mesh>
  void updateMesh(Mesh& mesh, const Eigen::Isometry3f* transform = nullptr) const;

  void updateVertices(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                      std::vector<Timestamp>* stamps,
                      std::vector<uint32_t>* semantics = nullptr,
                      const Eigen::Isometry3f* transform = nullptr) const;

  void updateMesh(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                  std::vector<Timestamp>& stamps,
                  std::vector<pcl::Vertices>& faces,
                  std::vector<uint32_t>* semantics = nullptr,
                  const Eigen::Isometry3f* transform = nullptr) const;

  size_t addVertex(Timestamp timestamp_ns,
                   const pcl::PointXYZRGBA& point,
                   std::optional<uint32_t> semantics = std::nullopt,
                   bool archive = false);

  void addFace(const Face& face, bool archive = false);

  bool hasSemantics() const;

  size_t getNumArchivedVertices() const;

  size_t getNumArchivedFaces() const;

  size_t getTotalArchivedVertices() const;

  size_t getTotalArchivedFaces() const;

  size_t getTotalVertices() const;

  size_t getTotalFaces() const;

  size_t getLocalIndex(size_t index) const;

  size_t getGlobalIndex(size_t index) const;

  pcl::IndicesPtr getActiveIndices() const;

  void checkFaces(const std::string& name) const;

  void validate(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                std::vector<Timestamp>& stamps,
                std::vector<pcl::Vertices>& faces,
                std::vector<uint32_t>* semantics = nullptr) const;

  size_t vertex_start = 0;
  size_t face_start = 0;

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertex_updates;
  std::vector<Timestamp> stamp_updates;
  std::vector<uint32_t> semantic_updates;
  std::vector<Face> face_updates;
  std::vector<Face> face_archive_updates;
  std::map<size_t, size_t> prev_to_curr;
  std::set<size_t> deleted_indices;
  std::set<size_t> observed_indices;
  std::set<size_t> new_indices;

 protected:
  size_t num_archived_vertices_ = 0;
};

std::ostream& operator<<(std::ostream& out, const MeshDelta& delta);

template <typename Vertices>
void MeshDelta::updateVertices(Vertices& vertices,
                               const Eigen::Isometry3f* transform) const {
  const bool use_semantics = hasSemantics();
  const size_t total_vertices = vertex_start + vertex_updates->size();
  traits::resize_vertices(vertices, total_vertices);

  for (size_t i = 0; i < vertex_updates->size(); ++i) {
    const size_t idx = i + vertex_start;
    const auto& p = vertex_updates->at(i);
    traits::Pos pos;
    if (transform) {
      pos = *transform * traits::Pos(p.x, p.y, p.z);
    } else {
      pos = traits::Pos(p.x, p.y, p.z);
    }
    traits::VertexTraits traits;
    traits.color = {p.r, p.g, p.b, p.a};
    traits.stamp = stamp_updates.at(i);
    if (use_semantics) {
      traits.label = semantic_updates.at(i);
    }
    traits::set_vertex(vertices, idx, pos, traits);
  }
}

template <typename Faces>
void MeshDelta::updateFaces(Faces& faces) const {
  const size_t total_faces =
      face_start + face_archive_updates.size() + face_updates.size();
  traits::resize_faces(faces, total_faces);

  size_t face_idx = face_start;
  for (const auto& face : face_archive_updates) {
    traits::set_face(faces, face_idx, {{face.v1, face.v2, face.v3}});
    ++face_idx;
  }

  for (const auto& face : face_updates) {
    traits::set_face(faces, face_idx, {{face.v1, face.v2, face.v3}});
    ++face_idx;
  }
}

template <typename Vertices, typename Faces>
void MeshDelta::updateMesh(Vertices& vertices,
                           Faces& faces,
                           const Eigen::Isometry3f* transform) const {
  updateVertices<Vertices>(vertices, transform);
  updateFaces<Faces>(faces);
}

template <typename Mesh>
void MeshDelta::updateMesh(Mesh& mesh, const Eigen::Isometry3f* transform) const {
  // dispatch for types implementing faces and vertices adl api
  updateMesh(mesh, mesh, transform);
}

template <typename Vertices, typename Faces>
MeshDelta::Ptr MeshDelta::fromMesh(const Vertices& vertices, const Faces& faces) {
  auto delta = std::make_shared<MeshDelta>(0, 0);

  const auto num_vertices = traits::num_vertices(vertices);
  for (size_t i = 0; i < num_vertices; ++i) {
    traits::VertexTraits traits;
    const auto pos = traits::get_vertex(vertices, i, &traits);
    auto& point = delta->vertex_updates->emplace_back();
    point.x = pos.x();
    point.y = pos.y();
    point.z = pos.z();
    if (traits.color) {
      const auto& c = *traits.color;
      point.r = c[0];
      point.g = c[1];
      point.b = c[2];
      point.a = c[3];
    }

    if (traits.stamp) {
      delta->stamp_updates.push_back(*traits.stamp);
    }

    if (traits.label) {
      delta->semantic_updates.push_back(*traits.label);
    }
  }

  const auto num_faces = traits::num_faces(faces);
  for (size_t i = 0; i < num_faces; ++i) {
    delta->face_updates.push_back(traits::get_face(faces, i));
  }

  return delta;
}

template <typename Mesh>
MeshDelta::Ptr MeshDelta::fromMesh(const Mesh& mesh) {
  return fromMesh(mesh, mesh);
}

inline size_t pgmoNumVertices(const MeshDelta& delta) {
  return delta.vertex_updates ? delta.vertex_updates->size() : 0;
}

inline traits::Pos pgmoGetVertex(const MeshDelta& delta,
                                 size_t i,
                                 traits::VertexTraits* traits) {
  const auto& point = delta.vertex_updates->at(i);
  traits::Pos pos(point.x, point.y, point.z);
  if (!traits) {
    return pos;
  }

  traits->color = {point.r, point.g, point.b, point.a};
  if (i <= delta.stamp_updates.size()) {
    traits->stamp = delta.stamp_updates[i];
  }

  if (i <= delta.semantic_updates.size()) {
    traits->label = delta.semantic_updates[i];
  }
  return pos;
}

inline size_t pgmoNumFaces(const MeshDelta& delta) {
  return delta.face_updates.size() + delta.face_archive_updates.size();
}

inline traits::Face pgmoGetFace(const MeshDelta& delta, size_t i) {
  if (i < delta.face_archive_updates.size()) {
    return delta.face_archive_updates[i];
  }

  i -= delta.face_archive_updates.size();
  return delta.face_updates.at(i);
}

}  // namespace kimera_pgmo
