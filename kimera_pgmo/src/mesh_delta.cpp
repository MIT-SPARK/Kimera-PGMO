/**
 * @file   mesh_delta.cpp
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#include "kimera_pgmo/mesh_delta.h"

namespace kimera_pgmo {

size_t pgmoNumVertices(const MeshDelta& delta) { return delta.getNumVertices(); }

traits::Pos pgmoGetVertex(const MeshDelta& delta,
                          size_t i,
                          traits::VertexTraits* traits) {
  const auto& point = delta.getVertex(i);
  if (traits) {
    *traits = point.traits;
  }

  return point.pos;
}

size_t pgmoNumFaces(const MeshDelta& delta) { return delta.getNumFaces(); }

traits::Face pgmoGetFace(const MeshDelta& delta, size_t i) { return delta.getFace(i); }

MeshDelta::MeshDelta(traits::Timestamp timestamp_ns, uint16_t sequence_number)
    : timestamp_ns(timestamp_ns), sequence_number(sequence_number) {}

size_t MeshDelta::addVertex(const traits::Pos& pos,
                            const traits::VertexTraits* traits,
                            bool archive) {
  const size_t index = vertex_updates_.size();
  if (archive) {
    ++num_archived_vertices_;
  }

  auto& update = vertex_updates_.emplace_back();
  update.pos = pos;
  if (traits) {
    update.traits = *traits;
  }

  return index;
}

void MeshDelta::addFace(const Face& face, bool archive) {
  if (archive) {
    face_archive_updates_.push_back(face);
  } else {
    face_updates_.push_back(face);
  }
}

size_t MeshDelta::getNumVertices() const { return vertex_updates_.size(); }

size_t MeshDelta::getNumArchivedVertices() const { return num_archived_vertices_; }

size_t MeshDelta::getNumFaces() const {
  return face_updates_.size() + face_archive_updates_.size();
}

size_t MeshDelta::getNumArchivedFaces() const { return face_archive_updates_.size(); }

const MeshDelta::Vertex& MeshDelta::getVertex(size_t i) const {
  return vertex_updates_.at(i);
}

const MeshDelta::Face& MeshDelta::getFace(size_t i) const {
  if (i < face_archive_updates_.size()) {
    return face_archive_updates_[i];
  }

  i -= face_archive_updates_.size();
  return face_updates_.at(i);
}

}  // namespace kimera_pgmo
