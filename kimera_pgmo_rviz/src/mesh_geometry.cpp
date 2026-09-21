#include "kimera_pgmo_rviz/mesh_geometry.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <unordered_map>

namespace kimera_pgmo {
namespace {

bool validFace(const traits::Face& face, const std::vector<traits::Vertex>& vertices) {
  for (const auto index : face) {
    if (index >= vertices.size() || !vertices[index].pos.allFinite()) {
      return false;
    }
  }

  return true;
}

}  // namespace

void MeshGeometry::setMesh(std::vector<traits::Vertex> vertices,
                           std::vector<traits::Face> faces) {
  vertices_ = std::move(vertices);
  faces_ = std::move(faces);
  offsets_ = {};
  sequence_number_.reset();
  archived_chunks_ = 0;
  // Old vertex indices need not exist in the replacement mesh.
  chunks_.clear();
  vertex_chunks_.clear();
  normals_.assign(normals_enabled_ ? vertices_.size() : 0, Eigen::Vector3f::Zero());
  std::set<size_t> changed;
  updateNormals(0, 1.0f, changed);
  rebuildChunks(0);
}

bool MeshGeometry::needsRecovery(uint16_t sequence_number) const {
  if (!sequence_number_) {
    return false;
  }

  const auto expected = static_cast<uint16_t>(*sequence_number_ + 1);
  return sequence_number != expected;
}

void MeshGeometry::validateDelta(const MeshDelta& delta) const {
  // An empty mesh can start receiving deltas midway through a stream.
  const auto active_vertices = vertices_.size() - offsets_.archived_vertices;
  if (!vertices_.empty() && delta.info.prev_active_vertices != active_vertices) {
    throw std::invalid_argument("Mesh delta does not match the active vertex count");
  }

  // The unarchived suffix includes pending faces as well as active faces.
  const auto unarchived_faces = faces_.size() - offsets_.archived_faces;
  if (!faces_.empty() && delta.info.prev_active_faces > unarchived_faces) {
    throw std::invalid_argument("Mesh delta active faces overlap the archive boundary");
  }
}

size_t MeshGeometry::applyDelta(const MeshDelta& delta) {
  const auto recover = needsRecovery(delta.info.sequence_number);
  if (!recover) {
    validateDelta(delta);
  }

  const auto first_vertex = offsets_.archived_vertices;
  const auto first_face = offsets_.archived_faces;
  std::set<size_t> changed;
  updateNormals(first_face, -1.0f, changed);
  if (recover) {
    // Pending faces also depend on the missing active-vertex remapping.
    vertices_.resize(first_vertex);
    faces_.resize(first_face);
  }

  const MeshDelta::TrackingInfo no_history;
  const auto& tracking = recover ? no_history : delta.info;
  delta.updateMesh(vertices_, faces_, offsets_, tracking);
  if (normals_enabled_) {
    normals_.resize(vertices_.size(), Eigen::Vector3f::Zero());
    // No fully archived face references the replaced active vertex suffix.
    std::fill(normals_.begin() + first_vertex, normals_.end(), Eigen::Vector3f::Zero());
  }

  updateNormals(first_face, 1.0f, changed);

  // Refill the unfinished archive tail and rebuild pending/active faces. Earlier
  // archive chunks keep their geometry and GPU buffers.
  rebuildChunks(archived_chunks_ ? archived_chunks_ - 1 : 0);
  dirtyNormals(changed);
  sequence_number_ = delta.info.sequence_number;
  return first_vertex;
}

void MeshGeometry::setMaxVertices(size_t max_vertices) {
  if (max_vertices < 3) {
    throw std::invalid_argument("A mesh chunk must fit at least one triangle");
  }

  if (max_vertices_ == max_vertices) {
    return;
  }

  max_vertices_ = max_vertices;
  rebuildChunks(0);
}

void MeshGeometry::setNormalsEnabled(bool enabled) {
  if (normals_enabled_ == enabled) {
    return;
  }

  normals_enabled_ = enabled;
  normals_.assign(enabled ? vertices_.size() : 0, Eigen::Vector3f::Zero());
  std::set<size_t> changed;
  updateNormals(0, 1.0f, changed);
  markAllDirty();
}

void MeshGeometry::updateNormals(size_t first_face,
                                 float sign,
                                 std::set<size_t>& changed) {
  if (!normals_enabled_) {
    return;
  }

  for (size_t i = first_face; i < faces_.size(); ++i) {
    const auto& face = faces_[i];
    if (!validFace(face, vertices_)) {
      continue;
    }

    const auto& a = vertices_[face[0]].pos;
    const auto& b = vertices_[face[1]].pos;
    const auto& c = vertices_[face[2]].pos;
    const Eigen::Vector3f cross = (b - a).cross(c - a);
    const auto length = cross.norm();
    if (!std::isfinite(length) || length <= 1.0e-12f) {
      continue;
    }

    const Eigen::Vector3f contribution = sign * cross / length;
    for (const auto index : face) {
      normals_[index] += contribution;
      changed.insert(index);
    }
  }
}

Eigen::Vector3f MeshGeometry::normal(size_t vertex) const {
  if (!normals_enabled_ || normals_[vertex].squaredNorm() <= 1.0e-12f) {
    return Eigen::Vector3f::UnitZ();
  }

  return normals_[vertex].normalized();
}

void MeshGeometry::rebuildChunks(size_t keep) {
  const auto first = keep < chunks_.size() ? chunks_[keep].first_face : 0;
  for (size_t i = keep; i < chunks_.size(); ++i) {
    for (const auto vertex : chunks_[i].vertices) {
      auto& owners = vertex_chunks_[vertex];
      owners.erase(std::lower_bound(owners.begin(), owners.end(), keep), owners.end());
    }
  }

  chunks_.resize(keep);
  vertex_chunks_.resize(vertices_.size());
  appendChunks(first, offsets_.archived_faces);
  archived_chunks_ = chunks_.size();
  appendChunks(std::max(first, offsets_.archived_faces), faces_.size());
}

void MeshGeometry::appendChunks(size_t first, size_t end) {
  std::unordered_map<size_t, size_t> local_indices;
  bool need_chunk = true;
  auto chunk_start = first;
  for (size_t i = first; i < end; ++i) {
    const auto& face = faces_[i];
    if (!validFace(face, vertices_)) {
      continue;
    }

    size_t added = 0;
    for (const auto vertex : face) {
      if (!local_indices.count(vertex)) {
        ++added;
      }
    }

    if (!need_chunk && (local_indices.size() + added > max_vertices_ ||
                        chunks_.back().faces.size() >= 2 * max_vertices_)) {
      need_chunk = true;
      chunk_start = i;
    }

    if (need_chunk) {
      chunks_.emplace_back();
      chunks_.back().first_face = chunk_start;
      local_indices.clear();
      need_chunk = false;
    }

    auto& chunk = chunks_.back();
    traits::Face local_face;
    for (size_t j = 0; j < face.size(); ++j) {
      const auto [iter, inserted] =
          local_indices.emplace(face[j], chunk.vertices.size());
      if (inserted) {
        chunk.vertices.push_back(face[j]);
        vertex_chunks_[face[j]].push_back(chunks_.size() - 1);
      }

      local_face[j] = iter->second;
    }

    chunk.faces.push_back(local_face);
  }
}

void MeshGeometry::dirtyNormals(const std::set<size_t>& changed) {
  for (const auto vertex : changed) {
    if (vertex >= vertex_chunks_.size()) {
      continue;
    }

    for (const auto chunk : vertex_chunks_[vertex]) {
      chunks_[chunk].dirty = true;
    }
  }
}

void MeshGeometry::markAllDirty() {
  for (auto& chunk : chunks_) {
    chunk.dirty = true;
  }
}

void MeshGeometry::clearDirty() {
  for (auto& chunk : chunks_) {
    chunk.dirty = false;
  }
}

}  // namespace kimera_pgmo
