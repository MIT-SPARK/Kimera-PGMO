/**
 * @file   mesh_delta.cpp
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#include "kimera_pgmo/mesh_delta.h"

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

using TrackingInfo = MeshDelta::TrackingInfo;

MeshDelta::MeshDelta(const TrackingInfo& info) : info(info) {}

size_t MeshDelta::addVertex(const traits::Pos& pos,
                            const traits::VertexTraits& traits,
                            bool archive) {
  const size_t index = vertex_updates_.size();
  if (archive) {
    ++num_archived_vertices_;
  }

  vertex_properties_ |= traits.properties;
  vertex_updates_.push_back({pos, traits});
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

size_t MeshDelta::getNumActiveVertices() const {
  return vertex_updates_.size() - num_archived_vertices_;
}

size_t MeshDelta::getNumArchivedVertices() const { return num_archived_vertices_; }

size_t MeshDelta::getNumFaces() const {
  return face_updates_.size() + face_archive_updates_.size();
}

size_t MeshDelta::getNumActiveFaces() const { return face_updates_.size(); }

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

const std::vector<MeshDelta::Face>& MeshDelta::face_updates() const {
  return face_updates_;
}

const std::vector<MeshDelta::Face>& MeshDelta::face_archive_updates() const {
  return face_archive_updates_;
}

const std::map<size_t, size_t>& MeshDelta::prev_to_curr() const {
  return prev_to_curr_;
}

std::map<size_t, size_t>& MeshDelta::prev_to_curr() { return prev_to_curr_; }

const traits::VertexProperties& MeshDelta::vertex_properties() const {
  return vertex_properties_;
}

size_t pgmoNumVertices(const MeshDelta& delta) { return delta.getNumVertices(); }

traits::VertexProperties pgmoGetVertexProperties(const MeshDelta& delta) {
  return delta.vertex_properties();
}

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

}  // namespace kimera_pgmo
