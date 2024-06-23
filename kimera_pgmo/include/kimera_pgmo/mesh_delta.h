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
#include <vector>
#include <set>

#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo/pcl_mesh_traits.h"
#include "kimera_pgmo/utils/common_structs.h"

namespace kimera_pgmo {

struct Face {
  Face(size_t v1, size_t v2, size_t v3);

  Face(const std::vector<size_t>& indices, size_t i);

  bool valid() const;

  void fill(std::vector<uint32_t>& other) const;

  uint32_t v1;
  uint32_t v2;
  uint32_t v3;
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

  template <typename Vertices>
  void updateVertices(Vertices& vertices) const;

  template <typename Faces>
  void updateFaces(Faces& faces) const;

  template <typename Vertices, typename Faces>
  void updateMesh(Vertices& vertices, Faces& faces) const;

  template <typename Mesh>
  void updateMesh(Mesh& mesh) const;

  void updateVertices(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                      std::vector<Timestamp>* stamps,
                      std::vector<uint32_t>* semantics = nullptr) const;

  void updateMesh(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                  std::vector<Timestamp>& stamps,
                  std::vector<pcl::Vertices>& faces,
                  std::vector<uint32_t>* semantics = nullptr) const;

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
void MeshDelta::updateVertices(Vertices& vertices) const {
  const bool use_semantics = hasSemantics();
  const size_t total_vertices = vertex_start + vertex_updates->size();
  traits::resize_vertices(vertices, total_vertices);

  for (size_t i = 0; i < vertex_updates->size(); ++i) {
    const size_t idx = i + vertex_start;
    const auto& p = vertex_updates->at(i);
    const traits::Pos pos(p.x, p.y, p.z);
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
void MeshDelta::updateMesh(Vertices& vertices, Faces& faces) const {
  updateVertices<Vertices>(vertices);
  updateFaces<Faces>(faces);
}

template <typename Mesh>
void MeshDelta::updateMesh(Mesh& mesh) const {
  // dispatch for types implementing faces and vertices adl api
  updateMesh(mesh, mesh);
}

}  // namespace kimera_pgmo
