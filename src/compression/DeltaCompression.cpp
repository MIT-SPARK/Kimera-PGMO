/**
 * @file   DeltaCompression.cpp
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include "kimera_pgmo/compression/DeltaCompression.h"

#include <iterator>

#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/VoxbloxMsgInterface.h"

namespace kimera_pgmo {

using voxblox::BlockIndexList;

struct RedunancyChecker {
  using SparseAdjacencyMatrix = std::map<size_t, std::set<size_t>>;

  bool check(const Face& face) const {
    const bool present = hasEdge(face.v1, face.v2) && hasEdge(face.v2, face.v3) &&
                         hasEdge(face.v2, face.v3);
    return !present;
  }

  void add(const Face& face) {
    addEdge(face.v1, face.v2);
    addEdge(face.v2, face.v3);
    addEdge(face.v3, face.v1);
  }

  bool hasEdge(size_t source, size_t target) const {
    const auto iter = A.find(source);
    if (iter == A.end()) {
      return false;
    }

    return iter->second.count(target);
  }

  void addEdge(size_t source, size_t target) {
    auto iter = A.find(source);
    if (iter == A.end()) {
      iter = A.insert({source, {}}).first;
    }

    iter->second.insert(target);
  }

  SparseAdjacencyMatrix A;
};

size_t getRemappedIndex(const std::map<size_t, size_t>& remapping, size_t original) {
  const auto iter = remapping.find(original);
  return iter == remapping.end() ? original : iter->second;
}

void VertexInfo::addObservation() const {
  ++active_refs;
  ++inactive_refs;
}

void VertexInfo::archiveObservation() const { --active_refs; }

void VertexInfo::removeObservation() const {
  --active_refs;
  --inactive_refs;
}

bool VertexInfo::notObserved() const { return inactive_refs <= 0; }

bool VertexInfo::shouldArchive() const { return active_refs <= 0; }

DeltaCompression::DeltaCompression(double resolution)
    : resolution_(resolution),
      index_scale_(1.0 / resolution),
      num_archived_vertices_(0),
      num_archived_faces_(0) {}

void DeltaCompression::addPoint(const pcl::PointXYZRGBA& point,
                                uint64_t timestamp_ns,
                                std::vector<size_t>& face_map,
                                std::vector<size_t>& active_remapping,
                                voxblox::LongIndexSet& curr_voxels) {
  const voxblox::LongIndex vertex_index(std::round(point.x * index_scale_),
                                        std::round(point.y * index_scale_),
                                        std::round(point.z * index_scale_));

  auto info_iter = vertices_map_.find(vertex_index);
  if (info_iter == vertices_map_.end()) {
    info_iter = vertices_map_.insert({vertex_index, {}}).first;
    // force update for a new point later (timestamp gets overwritten with correct
    // timestamp)
    info_iter->second.timestamp_ns = timestamp_ns + 1;
  }

  auto& info = info_iter->second;
  if (info.timestamp_ns != timestamp_ns) {
    info.timestamp_ns = timestamp_ns;
    info.point = point;
    const size_t prev_index = info.mesh_index;
    info.mesh_index = active_remapping.size();
    active_remapping.push_back(prev_index);  // cache previous index
  }

  face_map.push_back(info.mesh_index);

  if (!curr_voxels.count(vertex_index)) {
    info.addObservation();  // add one observation per block
    curr_voxels.insert(vertex_index);
  }
}

void DeltaCompression::removeBlockObservations(const voxblox::LongIndexSet& to_remove) {
  for (const auto& prev : to_remove) {
    const auto& info = vertices_map_.at(prev);
    info.removeObservation();

    if (info.notObserved()) {
      delta_->deleted_indices.insert(info.mesh_index);
      vertices_map_.erase(prev);  // no blocks point to the vertex, so delete
      continue;
    }

    if (!info.shouldArchive()) {
      continue;  // nothing else to do
    }

    // we can't observe a vertex and archive it in the same pass,
    // so info.mesh_index should point to the previous index
    const size_t mesh_index = delta_->addVertex(info.timestamp_ns, info.point, true);
    delta_->prev_to_curr[info.mesh_index] = mesh_index;
    vertices_map_.erase(prev);
  }
}

void DeltaCompression::addActiveVertices(uint64_t timestamp_ns,
                                         std::vector<size_t>& active_remapping) {
  for (auto& id_info_pair : vertices_map_) {
    // vertices here are guaranteed to be unique
    auto& info = id_info_pair.second;
    const size_t mesh_index = delta_->addVertex(info.timestamp_ns, info.point);

    if (info.timestamp_ns == timestamp_ns) {
      if (!info.is_new) {
        // we use the active remapping slot as temporary variable to cache
        // the index in the previous delta
        const size_t prev_mesh_index = active_remapping[info.mesh_index];
        delta_->prev_to_curr[prev_mesh_index] = mesh_index;
      } else {
        info.is_new = false;
      }

      // set to correct delta index for face construction
      active_remapping[info.mesh_index] = mesh_index;
    } else {
      // if we haven't seen this vertex in this pass, add to prev_to_curr map
      delta_->prev_to_curr[info.mesh_index] = mesh_index;
    }

    // set mesh index to point to the correct index in the current delta
    info.mesh_index = mesh_index;
  }
}

void DeltaCompression::addActiveFaces(uint64_t timestamp_ns,
                                      const std::vector<size_t>& active_remapping,
                                      VoxbloxIndexMapping* remapping) {
  const auto& prev_to_curr = delta_->prev_to_curr;
  // note that we only need to check for duplicates per each "type" of face
  RedunancyChecker checker;
  for (auto& id_info_pair : block_info_map_) {
    auto& block_info = id_info_pair.second;
    const bool was_updated = block_info.update_time == timestamp_ns;

    IndexMapping* block_remap = nullptr;
    if (remapping && was_updated) {
      block_remap = &(remapping->insert({id_info_pair.first, {}}).first->second);
    }

    auto& indices = block_info.indices;
    for (size_t i = 0; i < indices.size(); i += 3) {
      if (was_updated) {
        // face points to active vertices, so grab actual delta index and add to
        // remapping
        indices[i] = active_remapping[indices[i]];
        indices[i + 1] = active_remapping[indices[i + 1]];
        indices[i + 2] = active_remapping[indices[i + 2]];
      } else {
        indices[i] = prev_to_curr.at(indices[i]);
        indices[i + 1] = prev_to_curr.at(indices[i + 1]);
        indices[i + 2] = prev_to_curr.at(indices[i + 2]);
      }

      if (block_remap) {
        // block_remap is only valid if remapping is provided and the block was updated
        // from this compression pass
        block_remap->insert({i, indices[i]});
        block_remap->insert({i + 1, indices[i + 1]});
        block_remap->insert({i + 2, indices[i + 2]});
      }

      const Face face(indices, i);
      if (!face.valid()) {
        continue;
      }

      if (!checker.check(face)) {
        continue;
      }

      checker.add(face);
      delta_->addFace(face);
    }
  }
}

MeshDelta::Ptr DeltaCompression::update(const voxblox_msgs::Mesh& mesh,
                                        uint64_t timestamp_ns,
                                        VoxbloxIndexMapping* remapping) {
  VoxbloxMsgInterface interface(&mesh);
  return update(mesh, timestamp_ns, remapping);
}

MeshDelta::Ptr DeltaCompression::update(MeshInterface& mesh,
                                        uint64_t timestamp_ns,
                                        VoxbloxIndexMapping* remapping) {
  if (archive_delta_) {
    delta_ = archive_delta_;
    archive_delta_.reset();
  } else {
    delta_.reset(new MeshDelta(num_archived_vertices_, num_archived_faces_));
  }

  updateRemapping(mesh, timestamp_ns, remapping);
  addPartialFaces();
  num_archived_faces_ = delta_->getTotalArchivedFaces();

  return delta_;
}

void DeltaCompression::updateRemapping(MeshInterface& mesh,
                                       uint64_t stamp_ns,
                                       VoxbloxIndexMapping* remapping) {
  std::vector<size_t> active_remapping;
  for (const auto& block_index : mesh.blockIndices()) {
    bool is_block_new = false;
    auto block_iter = block_info_map_.find(block_index);
    if (block_iter == block_info_map_.end()) {
      is_block_new = true;
      block_iter = block_info_map_.insert({block_index, {{}, stamp_ns, {}}}).first;
    }

    auto& block_info = block_iter->second;
    block_info.update_time = stamp_ns;
    block_info.indices.clear();

    voxblox::LongIndexSet curr_voxels;
    mesh.markBlockActive(block_index);
    for (size_t i = 0; i < mesh.activeBlockSize(); ++i) {
      addPoint(mesh.getActiveVertex(i),
               stamp_ns,
               block_info.indices,
               active_remapping,
               curr_voxels);
    }

    if (!is_block_new) {
      removeBlockObservations(block_info.vertices);
    }

    block_info.vertices = curr_voxels;
  }

  num_archived_vertices_ = delta_->getTotalArchivedVertices();

  addActiveVertices(stamp_ns, active_remapping);
  addActiveFaces(stamp_ns, active_remapping, remapping);
}

void DeltaCompression::pruneStoredMesh(uint64_t earliest_time_ns) {
  BlockIndexList to_clear;
  for (const auto& id_info_pair : block_info_map_) {
    if (id_info_pair.second.update_time <= earliest_time_ns) {
      to_clear.push_back(id_info_pair.first);
    }
  }

  pruneMeshBlocks(to_clear);
}

void DeltaCompression::clearArchivedBlocks(const BlockIndexList& to_clear) {
  pruneMeshBlocks(to_clear);
}

void DeltaCompression::pruneMeshBlocks(const BlockIndexList& to_clear) {
  archive_delta_.reset(new MeshDelta(num_archived_vertices_, num_archived_faces_));

  BlockIndexList valid_blocks;

  std::map<size_t, size_t> remapping;
  for (const auto& idx : to_clear) {
    auto block_iter = block_info_map_.find(idx);
    if (block_iter == block_info_map_.end()) {
      // TODO(nathan) warn about pruning unknown block
      continue;
    }

    valid_blocks.push_back(idx);
    const auto& block_info = block_iter->second;
    for (const auto& voxel : block_info.vertices) {
      auto& info = vertices_map_[voxel];
      info.archiveObservation();

      if (!info.shouldArchive()) {
        continue;
      }

      const size_t new_index =
          archive_delta_->addVertex(info.timestamp_ns, info.point, true);
      remapping[info.mesh_index] = new_index;
      archive_delta_->prev_to_curr[info.mesh_index] = new_index;
      vertices_map_.erase(voxel);
    }
  }

  num_archived_vertices_ = archive_delta_->getTotalArchivedVertices();

  // remap previously partially archived faces with any newly archived vertices and add
  // to delta if they become fully archived. note that they cannot be duplicates of any
  // faces added in the next for-loop, as they must point to at least one vertex that
  // was archived before this pass that no other active face points to (active faces
  // cannot point at archived vertices)
  updateArchivedFaces(remapping);

  // fully archive any face in to_clear that only points to archived vertices, and queue
  // any partially archived face
  archiveBlockFaces(valid_blocks, remapping);

  num_archived_faces_ = archive_delta_->getTotalArchivedFaces();

  for (const auto& idx : valid_blocks) {
    block_info_map_.erase(idx);
  }
}

void DeltaCompression::addPartialFaces() {
  RedunancyChecker checker;
  const auto& prev_to_curr = delta_->prev_to_curr;

  auto iter = archived_faces_.begin();
  while (iter != archived_faces_.end()) {
    auto& face = *iter;
    if (face.v1 >= num_archived_vertices_) {
      face.v1 = getRemappedIndex(prev_to_curr, face.v1);
    }

    if (face.v2 >= num_archived_vertices_) {
      face.v2 = getRemappedIndex(prev_to_curr, face.v2);
    }

    if (face.v3 >= num_archived_vertices_) {
      face.v3 = getRemappedIndex(prev_to_curr, face.v3);
    }

    if (!face.valid()) {
      continue;
    }

    if (!checker.check(face)) {
      continue;
    }

    checker.add(face);

    const bool can_archive = canBeArchived(face);
    delta_->addFace(face, can_archive);
    if (can_archive) {
      iter = archived_faces_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void DeltaCompression::updateArchivedFaces(const std::map<size_t, size_t>& remapping) {
  RedunancyChecker checker;

  auto iter = archived_faces_.begin();
  while (iter != archived_faces_.end()) {
    Face& face = *iter;
    face.v1 = getRemappedIndex(remapping, face.v1);
    face.v2 = getRemappedIndex(remapping, face.v2);
    face.v3 = getRemappedIndex(remapping, face.v3);

    if (!canBeArchived(face)) {
      ++iter;
      continue;
    }

    // remove face from list to make use of continue easier
    Face to_archive = face;
    iter = archived_faces_.erase(iter);

    if (!to_archive.valid()) {
      continue;
    }

    if (!checker.check(to_archive)) {
      continue;
    }

    checker.add(to_archive);
    archive_delta_->addFace(to_archive, true);
  }
}

bool DeltaCompression::canBeArchived(const Face& face) const {
  return face.v1 < num_archived_vertices_ && face.v2 < num_archived_vertices_ &&
         face.v3 < num_archived_vertices_;
}

void DeltaCompression::archiveBlockFaces(const BlockIndexList& to_clear,
                                         const std::map<size_t, size_t>& remapping) {
  // note that we only need to check for duplicates per each "type" of face
  RedunancyChecker checker;
  for (const auto& idx : to_clear) {
    const auto& indices = block_info_map_.at(idx).indices;
    for (size_t i = 0; i < indices.size(); i += 3) {
      const Face face(getRemappedIndex(remapping, indices.at(i)),
                      getRemappedIndex(remapping, indices.at(i + 1)),
                      getRemappedIndex(remapping, indices.at(i + 2)));

      if (!face.valid()) {
        continue;
      }

      if (!canBeArchived(face)) {
        archived_faces_.push_back(face);
        continue;
      }

      if (!checker.check(face)) {
        continue;
      }

      checker.add(face);
      archive_delta_->addFace(face, true);
    }
  }
}

}  // namespace kimera_pgmo
