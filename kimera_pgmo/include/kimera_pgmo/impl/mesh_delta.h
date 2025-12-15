#pragma once
#include "kimera_pgmo/mesh_delta.h"
#include "kimera_pgmo/mesh_traits.h"
#include <Eigen/Geometry>

namespace kimera_pgmo {

inline traits::Face offsetFace(const traits::Face& face, size_t offset) {
  return {face[0] + offset, face[1] + offset, face[2] + offset};
}

template <typename Vertices, typename Faces>
MeshDelta::Ptr MeshDelta::fromMesh(const Vertices& vertices, const Faces& faces) {
  auto delta = std::make_unique<MeshDelta>(MeshDelta::TrackingInfo{0, 0, 0});
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

template <template <typename T> typename ContainerT>
void MeshDelta::updateIndices(ContainerT<size_t>& indices,
                              const MeshOffsetInfo& offset,
                              bool* is_archived,
                              std::set<size_t>* deleted) const {
  if (is_archived) {
    *is_archived = true;
  }

  size_t index = 0;
  auto iter = indices.begin();
  while (iter != indices.end()) {
    if (*iter < offset.prev_archived_vertices) {
      ++iter;
      ++index;
      continue;
    }

    const auto local_idx = *iter - offset.prev_archived_vertices;
    auto remap = prev_to_curr_.find(local_idx);
    if (remap == prev_to_curr_.end()) {
      // deleted vertices won't be in remapping
      iter = indices.erase(iter);
      if (deleted) {
        deleted->insert(index);
      }

      ++index;
      continue;
    }

    *iter = remap->second + offset.prev_archived_vertices;
    if (is_archived && *iter >= offset.archived_vertices) {
      *is_archived = false;
    }

    ++iter;
    ++index;
  }
}

template <template <typename T> typename ContainerT>
ContainerT<size_t> MeshDelta::remapIndices(const ContainerT<size_t>& indices,
                                           const MeshOffsetInfo& offset,
                                           bool* is_archived,
                                           std::set<size_t>* deleted) const {
  if (is_archived) {
    *is_archived = true;
  }

  size_t index = 0;
  ContainerT<size_t> to_return;
  for (const auto global_idx : indices) {
    if (global_idx < offset.prev_archived_vertices) {
      to_return.push_back(global_idx);
      ++index;
      continue;
    }

    const auto local_idx = global_idx - offset.prev_archived_vertices;
    auto remap = prev_to_curr_.find(local_idx);
    if (remap == prev_to_curr_.end()) {
      // deleted vertices won't be in remapping
      if (deleted) {
        deleted->insert(index);
      }

      ++index;
      continue;
    }

    const auto new_idx = remap->second + offset.prev_archived_vertices;
    to_return.push_back(new_idx);
    if (is_archived && new_idx >= offset.archived_vertices) {
      *is_archived = false;
    }

    ++index;
  }

  return to_return;
}

template <typename Mesh>
MeshOffsetInfo MeshDelta::updateMesh(Mesh& mesh,
                                     const Eigen::Isometry3f* transform) const {
  // dispatch for types implementing faces and vertices adl api
  return updateMesh(mesh, mesh, transform);
}

template <typename Vertices, typename Faces>
MeshOffsetInfo MeshDelta::updateMesh(Vertices& vertices,
                                     Faces& faces,
                                     const Eigen::Isometry3f* transform) const {
  const auto offset = updateVertices<Vertices>(vertices, transform);
  updateFaces<Faces>(faces, offset);
  return {offset + num_archived_vertices_, offset, 0};
}

template <typename Vertices>
size_t MeshDelta::updateVertices(Vertices& vertices,
                                 const Eigen::Isometry3f* transform) const {
  const auto curr_size = traits::num_vertices(vertices);
  if (curr_size < info.prev_active_vertices) {
    throw std::logic_error("Invalid target vertices!");
  }

  const auto start_idx = curr_size - info.prev_active_vertices;
  const auto total_vertices = start_idx + vertex_updates_.size();
  traits::resize_vertices(vertices, total_vertices);

  for (size_t i = 0; i < vertex_updates_.size(); ++i) {
    const auto& p = vertex_updates_.at(i);
    traits::Pos pos = p.pos;
    if (transform) {
      pos = *transform * p.pos;
    }

    traits::set_vertex(vertices, start_idx + i, pos, &p.traits);
  }

  return start_idx;
}

template <typename Faces>
void MeshDelta::updateFaces(Faces& faces, size_t vertex_offset) const {
  const auto curr_size = traits::num_faces(faces);
  if (curr_size < info.prev_active_faces) {
    throw std::logic_error("Invalid target vertices!");
  }

  const auto start_idx = curr_size - info.prev_active_faces;
  const size_t total_faces = start_idx + getNumFaces();
  traits::resize_faces(faces, total_faces);

  size_t face_idx = 0;
  for (const auto& face : face_archive_updates_) {
    traits::set_face(faces, face_idx, offsetFace(face, vertex_offset));
    ++face_idx;
  }

  for (const auto& face : face_updates_) {
    traits::set_face(faces, face_idx, offsetFace(face, vertex_offset));
    ++face_idx;
  }
}

}  // namespace kimera_pgmo
