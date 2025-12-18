#pragma once
#include "kimera_pgmo/mesh_delta.h"
#include "kimera_pgmo/mesh_traits.h"
#include <Eigen/Geometry>

namespace kimera_pgmo {

inline traits::Face offsetFace(const traits::Face& face, size_t offset) {
  return {face[0] + offset, face[1] + offset, face[2] + offset};
}

inline bool allVerticesBelow(const traits::Face& face, size_t archive_threshold) {
  return face[0] < archive_threshold && face[1] < archive_threshold &&
         face[2] < archive_threshold;
}

inline traits::Face remapFace(const traits::Face& face,
                              size_t offset,
                              const std::map<size_t, size_t>& remap) {
  return {face[0] >= offset ? remap.at(face[0] - offset) + offset : face[0],
          face[1] >= offset ? remap.at(face[1] - offset) + offset : face[1],
          face[2] >= offset ? remap.at(face[2] - offset) + offset : face[2]};
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
                              MeshOffsetInfo::RemapInfo* info) const {
  size_t index = 0;
  auto iter = indices.begin();
  while (iter != indices.end()) {
    const auto remapped = remapIndex(offset, *iter);
    if (!remapped) {
      iter = indices.erase(iter);
      if (info) {
        info->deleted_indices.insert(index);
      }

      ++index;
      continue;
    }

    *iter = *remapped;
    if (info) {
      info->addIndex(*iter);
    }

    ++iter;
    ++index;
  }

  if (info) {
    info->all_archived = info->max_index < offset.archived_vertices;
  }
}

template <template <typename T> typename ContainerT>
ContainerT<size_t> MeshDelta::remapIndices(const ContainerT<size_t>& indices,
                                           const MeshOffsetInfo& offset,
                                           MeshOffsetInfo::RemapInfo* info) const {
  size_t index = 0;
  ContainerT<size_t> to_return;
  for (const auto global_idx : indices) {
    const auto remapped = remapIndex(offset, global_idx);
    if (!remapped) {
      if (info) {
        info->deleted_indices.insert(index);
      }

      ++index;
      continue;
    }

    const auto new_idx = remapped.value();
    to_return.push_back(new_idx);
    if (info) {
      info->addIndex(new_idx);
    }

    ++index;
  }

  if (info) {
    info->all_archived = info->max_index < offset.archived_vertices;
  }

  return to_return;
}

template <typename Mesh>
void MeshDelta::updateMesh(Mesh& mesh,
                           MeshOffsetInfo& offsets,
                           const Eigen::Isometry3f* transform) const {
  // dispatch for types implementing faces and vertices adl api
  updateMesh(mesh, mesh, offsets, transform);
}

template <typename Vertices, typename Faces>
void MeshDelta::updateMesh(Vertices& vertices,
                           Faces& faces,
                           MeshOffsetInfo& offsets,
                           const Eigen::Isometry3f* transform) const {
  const auto vertex_offset = updateVertices<Vertices>(vertices, transform);
  const auto archived_faces = updateFaces<Faces>(faces, offsets, vertex_offset);
  offsets = {
      vertex_offset + num_archived_vertices_,
      vertex_offset,
      archived_faces,
  };
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
size_t MeshDelta::updateFaces(Faces& faces,
                              const MeshOffsetInfo& prev_offsets,
                              size_t vertex_offset) const {
  const auto curr_size = traits::num_faces(faces);
  if (curr_size < info.prev_active_faces) {
    throw std::logic_error("Invalid target vertices!");
  }

  const size_t archived_threshold = vertex_offset + num_archived_vertices_;
  const auto start_idx = curr_size - info.prev_active_faces;
  const size_t total_faces = start_idx + getNumFaces();
  traits::resize_faces(faces, total_faces);

  // we first remap and sort any face between the last archived face offset and
  // the current insertion point for any active face (start_idx)
  size_t pending_start = prev_offsets.archived_faces;
  for (size_t i = pending_start; i < start_idx; ++i) {
    const auto f_p = traits::get_face(faces, i);
    const auto f_n = remapFace(f_p, vertex_offset, prev_to_curr_);
    if (!allVerticesBelow(f_n, archived_threshold)) {
      traits::set_face(faces, i, f_n);
      continue;
    }

    if (i != pending_start) {
      // swap previously processed pending face with current face if pending faces exist
      // between this index and the last archived face
      traits::set_face(faces, i, traits::get_face(faces, pending_start));
    }

    // move fully archived face to end of archived faces and move the pending band
    // forward one
    traits::set_face(faces, pending_start, f_n);
    ++pending_start;
  }

  // we sort any newly "archived face" into pending and fully-archived faces by
  // inserting them at the end of the archived band or the pending band
  size_t face_idx = start_idx;
  for (const auto& face : face_archive_updates_) {
    const auto f_n = offsetFace(face, vertex_offset);
    if (!allVerticesBelow(f_n, archived_threshold)) {
      // put pending face at end of pending band
      traits::set_face(faces, face_idx, offsetFace(face, vertex_offset));
      ++face_idx;
      continue;
    }

    if (face_idx != pending_start) {
      // swap previously processed pending face with current face if pending faces exist
      // between this index and the last archived face
      traits::set_face(faces, face_idx, traits::get_face(faces, pending_start));
    }

    traits::set_face(faces, pending_start, f_n);
    ++pending_start;
    ++face_idx;
  }

  // at this point, face_idx should be the end of the pending band, and anything active
  // gets inserted after
  for (const auto& face : face_updates_) {
    traits::set_face(faces, face_idx, offsetFace(face, vertex_offset));
    ++face_idx;
  }

  return pending_start;
}

}  // namespace kimera_pgmo
