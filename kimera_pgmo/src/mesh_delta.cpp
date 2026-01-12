/**
 * @file   mesh_delta.cpp
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#include "kimera_pgmo/mesh_delta.h"

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

using TrackingInfo = MeshDelta::TrackingInfo;

TrackingInfo TrackingInfo::with_remap(uint16_t sequence_number,
                                      size_t prev_active_vertices,
                                      size_t prev_active_faces,
                                      const std::map<size_t, size_t>& remap) {
  TrackingInfo info{sequence_number, prev_active_vertices, prev_active_faces};
  info.prev_to_curr = std::make_shared<std::map<size_t, size_t>>(remap);
  return info;
}

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

const MeshDelta::Vertex& MeshDelta::getVertex(size_t i) const {
  return vertex_updates_.at(i);
}

const MeshDelta::Face& MeshDelta::getFace(size_t i) const {
  if (i < face_archive_updates_.size()) {
    return face_archive_updates_[i];
  }

  return face_updates_.at(i - face_archive_updates_.size());
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

const traits::VertexProperties& MeshDelta::vertex_properties() const {
  return vertex_properties_;
}

const std::vector<MeshDelta::Face>& MeshDelta::faces() const { return face_updates_; }

const std::vector<MeshDelta::Face>& MeshDelta::archived_faces() const {
  return face_archive_updates_;
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
