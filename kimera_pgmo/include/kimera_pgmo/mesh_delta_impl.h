#pragma once
#include "kimera_pgmo/mesh_delta.h"
#include "kimera_pgmo/mesh_traits.h"
#include <Eigen/Geometry>

namespace kimera_pgmo {

template <typename Vertices>
void MeshDelta::updateVertices(Vertices& vertices,
                               const Eigen::Isometry3f* transform) const {
  // TODO(nathan) this is trickier than we though, the delta needs to know the change in
  // active vertices / the previous band sizes of the active and 1-hop vertices
  const size_t total_vertices = vertex_updates_.size();
  traits::resize_vertices(vertices, total_vertices);
  for (size_t i = 0; i < vertex_updates_.size(); ++i) {
    const auto& p = vertex_updates_.at(i);
    traits::Pos pos = p.pos;
    if (transform) {
      pos = *transform * p.pos;
    }

    // TODO(nathan) get indexing correct
    traits::set_vertex(vertices, i, pos, p.traits);
  }
}

template <typename Faces>
void MeshDelta::updateFaces(Faces& faces) const {
  // TODO(nathan) this is trickier than I thought
  const size_t total_faces = face_archive_updates_.size() + face_updates_.size();
  traits::resize_faces(faces, total_faces);

  size_t face_idx = 0;
  for (const auto& face : face_archive_updates_) {
    traits::set_face(faces, face_idx, face);
    ++face_idx;
  }

  for (const auto& face : face_updates_) {
    traits::set_face(faces, face_idx, face);
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
  auto delta = std::make_shared<MeshDelta>();
  const auto num_vertices = traits::num_vertices(vertices);
  for (size_t i = 0; i < num_vertices; ++i) {
    traits::VertexTraits traits;
    const auto pos = traits::get_vertex(vertices, i, &traits);
    delta->addVertex(pos, &traits);
  }

  const auto num_faces = traits::num_faces(faces);
  for (size_t i = 0; i < num_faces; ++i) {
    delta->face_updates_.push_back(traits::get_face(faces, i));
  }

  return delta;
}

template <typename Mesh>
MeshDelta::Ptr MeshDelta::fromMesh(const Mesh& mesh) {
  return fromMesh(mesh, mesh);
}

// vertex traits
size_t pgmoNumVertices(const MeshDelta& delta);
traits::Pos pgmoGetVertex(const MeshDelta& delta,
                          size_t i,
                          traits::VertexTraits* traits);

// face traits
size_t pgmoNumFaces(const MeshDelta& delta);
traits::Face pgmoGetFace(const MeshDelta& delta, size_t i);

}  // namespace kimera_pgmo
