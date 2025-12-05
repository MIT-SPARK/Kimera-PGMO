/**
 * @file   delta_compression.cpp
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include "kimera_pgmo/compression/delta_compression.h"

#include "kimera_pgmo/compression/redundancy_checker.h"
#include "kimera_pgmo/mesh_delta.h"
#include "kimera_pgmo/mesh_types.h"
#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {

using spatial_hash::BlockIndex;
using spatial_hash::BlockIndices;

namespace {

inline size_t addPointToDelta(MeshDelta& delta,
                              const VertexInfo& info,
                              bool should_archive = false) {
  return delta.addVertex(info.pos, info.traits, should_archive);
}

inline size_t getRemappedIndex(const std::map<size_t, size_t>& remapping,
                               size_t original) {
  const auto iter = remapping.find(original);
  // TODO(Yun): technically should always have key in remapping
  return iter == remapping.end() ? original : iter->second;
}

inline size_t getRemappedIndex(const std::map<size_t, size_t>& remapping,
                               size_t original,
                               size_t threshold) {
  if (original < threshold) {
    return original;
  }

  return getRemappedIndex(remapping, original);
}

inline bool allVerticesBelow(const traits::Face& face, size_t archive_threshold) {
  return face[0] < archive_threshold && face[1] < archive_threshold &&
         face[2] < archive_threshold;
}

inline void markBoundaryVertices(const traits::Face& face,
                                 const size_t num_archived,
                                 std::unordered_set<size_t>& pending) {
  if ((pending.count(face[0]) || face[0] < num_archived) &&
      (pending.count(face[1]) || face[1] < num_archived) &&
      (pending.count(face[2]) || face[2] < num_archived)) {
    // face points to vertices that are either archived or not archived
    return;
  }

  // at least one vertex points at an active vertex, don't archive anything yet
  pending.erase(face[0]);
  pending.erase(face[1]);
  pending.erase(face[2]);
}

/*inline std::string faceToStr(std::ostream& out, const traits::Face& face) {*/
/*std::stringstream ss;*/
/*ss << "(" << face[0] << ", " << face[1] << ", " << face[2] << ")";*/
/*return ss.str();*/
/*}*/

inline traits::Face faceFromIndices(const std::vector<size_t>& indices, size_t i) {
  return {indices.at(i), indices.at(i + 1), indices.at(i + 2)};
}

inline bool faceIsValid(const traits::Face& face) {
  return face[0] != face[1] && face[0] != face[2] && face[1] != face[2];
}

}  // namespace

void VertexInfo::addObservation() const { ++active_refs; }

void VertexInfo::removeObservation() const { --active_refs; }

void VertexInfo::archiveObservation() const {
  needs_archive = true;
  --active_refs;
}

bool VertexInfo::notObserved() const { return !needs_archive && active_refs <= 0; }

bool VertexInfo::shouldArchive() const { return needs_archive && active_refs <= 0; }

DeltaCompression::DeltaCompression(double resolution)
    : resolution_(resolution),
      index_scale_(1.0 / resolution),
      tracking_info_({1}),
      num_archived_vertices_(0),
      num_archived_faces_(0) {}

void DeltaCompression::addPoint(const traits::Pos& pos,
                                const traits::VertexTraits& traits,
                                std::vector<size_t>& face_map,
                                spatial_hash::LongIndexSet& curr_voxels) {
  // do voxel hashing at compression size to determine remapping to previous compressed
  // vertex (if it exists)
  const spatial_hash::LongIndex vertex_index(std::round(pos.x() * index_scale_),
                                             std::round(pos.y() * index_scale_),
                                             std::round(pos.z() * index_scale_));

  auto info_iter = vertices_map_.find(vertex_index);
  if (info_iter == vertices_map_.end()) {
    // update is forced by sequence number defaulting to -1
    info_iter = vertices_map_.insert({vertex_index, {}}).first;
  }

  auto& info = info_iter->second;
  // TODO(nathan) fix update
  info.pos = pos;
  info.traits = traits;
  if (info.sequence_number != tracking_info_.sequence_number) {
    const size_t prev_index = info.mesh_index;
    info.mesh_index = active_remapping_.size();
    active_remapping_.push_back(prev_index);  // cache previous index

    // mark vertex observed this pass
    info.sequence_number = tracking_info_.sequence_number;
  }

  face_map.push_back(info.mesh_index);
  if (!curr_voxels.count(vertex_index)) {
    info.addObservation();  // add one observation per block
    curr_voxels.insert(vertex_index);
  }
}

void DeltaCompression::removeBlockObservations(const LongIndexSet& to_remove) {
  // for every previous voxel observed by the block
  //   - remove an observation
  //   - if there are NO "inactive" (archived) blocks pointing to the voxel, register
  //   the voxed as deleted with the mesh delta and erase the voxel from the active
  //   voxel map
  //   - if there are other blocks still pointing to the voxel, do nothing
  //   - if there are "inactive" (archived) blocks pointing to the voxel,
  //      archive the voxel in the mesh delta and add the new index to the current
  //      remapping and erase the voxel from the active map
  for (const auto& prev : to_remove) {
    const auto& info = vertices_map_.at(prev);
    info.removeObservation();

    if (info.notObserved()) {  // inactive_refs <= 0
      delta_->deleted_indices_.insert(info.mesh_index);
      vertices_map_.erase(prev);
      continue;
    }

    if (!info.shouldArchive()) {
      continue;  // nothing else to do
    }

    // we can't observe a vertex and then need to archive it in the same pass, so
    // info.mesh_index should point to the previous index. Need to add vertex to
    // boundary list to be processed later
    archived_vertices_.push_back(info);
    vertices_map_.erase(prev);
  }
}

void DeltaCompression::addActiveVertices() {
  for (auto& [vertex_idx, info] : vertices_map_) {
    const auto new_idx = addPointToDelta(*delta_, info);
    if (info.sequence_number != tracking_info_.sequence_number) {
      // if we haven't seen this vertex in this pass, add to prev_to_curr_ map
      delta_->prev_to_curr_[info.mesh_index] = new_idx;
    } else {
      if (!info.is_new) {
        // we use the active remapping slot as temporary variable to cache
        // the index in the previous delta
        const size_t prev_mesh_index = active_remapping_[info.mesh_index];
        delta_->prev_to_curr_[prev_mesh_index] = new_idx;
      } else {
        delta_->new_indices_.insert(new_idx);
        info.is_new = false;
      }

      // set to correct delta index for face construction
      delta_->observed_indices_.insert(new_idx);
      active_remapping_[info.mesh_index] = new_idx;
    }

    // set mesh index to point to the correct index in the current delta
    info.mesh_index = new_idx;
  }
}

void DeltaCompression::addActiveFaces(HashedIndexMapping* remapping) {
  // for every current tracked block we
  //   - iterate through every "face" (set of 3 vertex indices)
  //   - if the face is from a block contained in the latest message, grab the new face
  //     indices from the remapping between the latest indices and their positions in
  //     the newest delta. Otherwise, carry any changes in indices between the last mesh
  //     delta and the newest one forward
  //  - store the remapping between every original vertex index in the latest message
  //    and the compressed vertex index in the latest delta (as the remapping is now
  //    fixed at this point, but was not when active vertices were being added)
  //  - add the face if it isn't degenerate or redundant
  const auto& prev_to_curr_ = delta_->prev_to_curr_;
  // note that we only need to check for duplicates per each "type" of face
  RedundancyChecker checker;
  for (auto& [idx, block_info] : block_info_map_) {
    const bool was_updated =
        block_info.sequence_number == tracking_info_.sequence_number;

    IndexMapping* block_remap = nullptr;
    if (remapping && was_updated) {
      block_remap = &(remapping->insert({idx, {}}).first->second);
    }

    auto& indices = block_info.indices;
    for (size_t i = 0; i < indices.size(); i += 3) {
      if (was_updated) {
        // face points to active vertices, so grab actual delta index and add to
        // remapping
        indices[i] = active_remapping_[indices[i]];
        indices[i + 1] = active_remapping_[indices[i + 1]];
        indices[i + 2] = active_remapping_[indices[i + 2]];
      } else {
        indices[i] = prev_to_curr_.at(indices[i]);
        indices[i + 1] = prev_to_curr_.at(indices[i + 1]);
        indices[i + 2] = prev_to_curr_.at(indices[i + 2]);
      }

      if (block_remap) {
        // block_remap is only valid if remapping is provided and the block was updated
        // from this compression pass
        block_remap->insert({i, indices[i]});
        block_remap->insert({i + 1, indices[i + 1]});
        block_remap->insert({i + 2, indices[i + 2]});
      }

      const auto face = faceFromIndices(indices, i);
      if (!faceIsValid(face)) {
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

void DeltaCompression::archiveBlocksByTime(uint64_t earliest_time_ns) {
  archiveBlocks([earliest_time_ns](const auto&, const auto& info) {
    return info.update_time_ns <= earliest_time_ns;
  });
}

void DeltaCompression::archiveBlocks(const BlockFilter& to_archive) {
  // 0. Only reset archive delta if necessary (to allow for multiple archive calls)
  if (!archive_delta_) {
    // TODO(nathan) need timestamp
    archive_delta_.reset(new MeshDelta(tracking_info_));
  }

  // 1(a). Populate current archived indices
  std::unordered_set<size_t> pending_vertices;
  for (const auto& pending : archived_vertices_) {
    pending_vertices.insert(pending.mesh_index);
  }

  // 1(b). Add newly archived vertices to archival candidates (and mark archived blocks)
  spatial_hash::BlockIndices to_erase;
  for (const auto& [idx, block_info] : block_info_map_) {
    if (!to_archive(idx, block_info)) {
      continue;
    }

    to_erase.push_back(idx);
    for (const auto& voxel : block_info.vertices) {
      auto& info = vertices_map_[voxel];
      info.archiveObservation();
      if (!info.shouldArchive()) {
        continue;
      }

      archived_vertices_.push_back(info);
      pending_vertices.insert(info.mesh_index);
      vertices_map_.erase(voxel);
    }
  }

  // 2. Mark vertices that can be archived by checking pending faces for archive
  for (const auto& idx : to_erase) {
    const auto& block_info = block_info_map_[idx];
    for (size_t i = 0; i < block_info.indices.size(); i += 3) {
      const auto face = faceFromIndices(block_info.indices, i);
      markBoundaryVertices(face, num_archived_vertices_, pending_vertices);
    }
  }

  for (const auto& face : archived_faces_) {
    markBoundaryVertices(face, num_archived_vertices_, pending_vertices);
  }

  // 3. Sweep archived vertices
  size_t boundary = archived_vertices_.size();
  for (size_t i = 0; i < boundary; ++i) {
    const auto& info = archived_vertices_[i];
    if (!pending_vertices.count(info.mesh_index)) {
      continue;
    }

    // add newly archived vertex to mesh delta
    const auto new_index = addPointToDelta(*archive_delta_, info, true);
    archive_delta_->prev_to_curr_[info.mesh_index] = new_index;
    // "delete" vertex by swapping to end and decreasing size
    std::swap(archived_vertices_[i], archived_vertices_[boundary - 1]);
    boundary -= 1;
    i -= 1;
  }

  // 4. Add boundary vertices to delta (allows archiving of faces pointing to pending or
  // archived vertices)
  archived_vertices_.resize(boundary);
  addPendingVertices(*archive_delta_);

  // 5. Sweep archived faces
  RedundancyChecker checker;
  std::vector<traits::Face> pending_faces;
  for (const auto& idx : to_erase) {
    archiveBlockFaces(block_info_map_.at(idx), checker, pending_faces);
    block_info_map_.erase(idx);
  }

  archive_delta_->face_archive_updates_.insert(
      archive_delta_->face_archive_updates_.end(),
      pending_faces.begin(),
      pending_faces.end());
  SPARK_LOG(DEBUG) << "Finished archive with delta containing "
                   << archive_delta_->vertex_updates_.size() << " vertice(s), "
                   << archive_delta_->face_updates_.size() << " face(s), and "
                   << archive_delta_->face_archive_updates_.size()
                   << " archived face(s) with "
                   << archive_delta_->getNumArchivedVertices()
                   << " vertice(s) being archived";
}

void DeltaCompression::archiveBlockFaces(const BlockInfo& block_info,
                                         RedundancyChecker& checker,
                                         std::vector<traits::Face>& pending_faces) {
  // We want to archive any face that points to an archived vertex or a pending vertex,
  // so we use the total vertices in the archive delta (which contains the new archived
  // vertices and the new pending vertices)
  // const auto archive_threshold = archive_delta_->getTotalVertices();
  // const auto pending_threshold = archive_delta_->getTotalArchivedVertices();
  const auto archive_threshold = 0;
  const auto pending_threshold = 0;

  // this remapping points from the previous active index to the current archival index
  // of all vertices archived from the block being archived. Any active vertex (or
  // previously archived vertex) will not be in the remapping
  const auto& indices = block_info.indices;
  const auto& prev_to_curr_ = archive_delta_->prev_to_curr_;
  for (size_t i = 0; i + 2 < indices.size(); i += 3) {
    auto face = faceFromIndices(indices, i);

    // prev_to_curr_.count(idx) checks that the vertex was added to
    // the archive delta, so this condition is saying that vertex index from the
    // previous update was not archived in this archival pass or a previous one.
    const auto can_archive = std::all_of(face.begin(), face.end(), [&](const auto v) {
      return prev_to_curr_.count(v) || v < num_archived_vertices_;
    });
    if (!can_archive) {
      // push any face that we can't deal with currently to be considered for archival
      // later. Crucially, we don't remap any face indices here as we can't
      // distinguish whether an index was remapped here or not
      archived_faces_.push_back(face);
      continue;
    }

    // remap face to respect newly archived vertices
    face[0] = getRemappedIndex(prev_to_curr_, face[0], num_archived_vertices_);
    face[1] = getRemappedIndex(prev_to_curr_, face[1], num_archived_vertices_);
    face[2] = getRemappedIndex(prev_to_curr_, face[2], num_archived_vertices_);
    if (!faceIsValid(face)) {
      continue;
    }

    if (!allVerticesBelow(face, archive_threshold)) {
      archived_faces_.push_back(face);
      continue;
    }

    if (!checker.check(face)) {
      continue;
    }

    checker.add(face);
    const auto fully_archived = allVerticesBelow(face, pending_threshold);
    if (fully_archived) {
      archive_delta_->addFace(face, true);
    } else {
      pending_faces.push_back(face);
    }
  }
}

void DeltaCompression::updateAndAddArchivedFaces() {
  // note that we only need to check for duplicates per each "type" of face
  RedundancyChecker checker;
  const auto& prev_to_curr_ = delta_->prev_to_curr_;
  // const auto archive_threshold = delta_->getTotalArchivedVertices();
  const auto archive_threshold = 0;

  auto iter = archived_faces_.begin();
  while (iter != archived_faces_.end()) {
    auto& face = *iter;
    face[0] = getRemappedIndex(prev_to_curr_, face[0], num_archived_vertices_);
    face[1] = getRemappedIndex(prev_to_curr_, face[1], num_archived_vertices_);
    face[1] = getRemappedIndex(prev_to_curr_, face[2], num_archived_vertices_);
    if (!faceIsValid(face)) {
      iter = archived_faces_.erase(iter);
      continue;
    }

    if (!checker.check(face)) {
      // if we hit this point, there is another face pending archive that contains the
      // same information. Regardless of whether or not the face is going to be archived
      // this pass, we should always drop the duplicates as the face will already be
      // covered
      iter = archived_faces_.erase(iter);
      continue;
    }

    const bool can_archive = allVerticesBelow(face, archive_threshold);
    checker.add(face);
    delta_->addFace(face, can_archive);
    if (can_archive) {
      iter = archived_faces_.erase(iter);
    } else {
      ++iter;
    }
  }
}

void DeltaCompression::addPendingVertices(MeshDelta& delta, size_t start_index) {
  // note that unlike the archived vertices, the mesh index of pending boundary vertices
  // changes every pass until they are actually fully archived
  for (size_t i = start_index; i < archived_vertices_.size(); ++i) {
    auto& info = archived_vertices_[i];
    const auto new_index = addPointToDelta(delta, info);
    delta.prev_to_curr_[info.mesh_index] = new_index;
    info.mesh_index = new_index;
  }
}

}  // namespace kimera_pgmo
