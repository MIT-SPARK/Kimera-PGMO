/**
 * @file   delta_compression.cpp
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include "kimera_pgmo/compression/delta_compression.h"

#include "kimera_pgmo/hashing.h"
#include "kimera_pgmo/utils/common_functions.h"
#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {
namespace {

inline void markBoundaryVertices(const traits::Face& face,
                                 std::unordered_set<size_t>& pending) {
  if (pending.count(face[0]) && pending.count(face[1]) && pending.count(face[2])) {
    // face points to vertices that are either archived or boundary
    return;
  }

  // at least one vertex points at an active vertex, don't archive anything yet
  pending.erase(face[0]);
  pending.erase(face[1]);
  pending.erase(face[2]);
}

std::string summarizeDelta(const MeshDelta& delta) {
  std::stringstream ss;
  ss << "delta containing " << delta.getNumVertices() << " vertice(s), "
     << delta.getNumActiveFaces() << " face(s), and " << delta.getNumArchivedFaces()
     << " archived face(s) with " << delta.getNumArchivedVertices()
     << " vertice(s) being archived";
  return ss.str();
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

void DefaultVertexUpdate::operator()(uint64_t timestamp_ns,
                                     const traits::Pos& pos,
                                     const traits::VertexTraits& traits,
                                     VertexInfo& info) const {
  info.pos = pos;
  info.traits.properties |= traits.properties;
  info.traits.properties.has_stamp = true;
  info.traits.properties.has_first_seen_stamp = true;
  info.traits.stamp = std::max(
      info.traits.stamp, traits.properties.has_stamp ? traits.stamp : timestamp_ns);
  info.traits.first_seen_stamp = std::min(
      info.traits.first_seen_stamp,
      traits.properties.has_first_seen_stamp ? traits.first_seen_stamp : timestamp_ns);

  if (traits.properties.has_color) {
    info.traits.color = traits.color;
  }

  if (traits.properties.has_label) {
    info.traits.label = traits.label;
  }
}

DeltaCompression::DeltaCompression(double resolution)
    : grid_(resolution), tracking_info_({1}) {}

MeshDelta::Ptr DeltaCompression::computeDelta(uint64_t timestamp_ns,
                                              HashedIndexMapping* remapping) {
  // Update happens independently of calls to archiveBlocks and archiveBlocksByTime. We
  // copy over the archive delta if it exists instead of making a new one.
  size_t prev_num_pending = 0;
  if (archive_delta_) {
    // we need to cache the amount of pending vertices we've already copied
    prev_num_pending = archive_delta_->getNumActiveVertices();
    delta_ = std::move(archive_delta_);
    archive_delta_.reset();  // technically not needed
  } else {
    delta_ = std::make_unique<MeshDelta>(tracking_info_);
  }

  // Applies any pending vertices that may be left over from old archival passes or from
  // removed block observations
  addPendingVertices(*delta_, prev_num_pending);

  // Add all active vertices and faces to the output mesh delta. This is any
  // compressed vertex that is the result of one or more blocks that hasn't
  // been archived and any face from any block that hasn't been archived.
  // note: this provides an invariant: every face that belongs to a block in
  // block_info_map_ has the correct indices after this is called and before the next
  // updateRemapping is called. This means that anyone archiving blocks can directly use
  // the faces without doing any remapping
  addActiveVertices(remapping);
  addActiveFaces();

  // Update all partially archived faces with the new remapping. This remaps
  // previously partially archived faces with any newly archived vertices and also
  // archives them in the delta if they only point at archived vertices. Note that they
  // cannot be duplicates of any active faces in the new mesh delta or any fully
  // archived faces in the cached mesh delta as they must point to at least one vertex
  // that was archived before this pass that no other active face points to and must
  // also point to at least one active vertex that was archived this pass.
  updateAndAddArchivedFaces();

  // Increment the pass number, finalize mesh delta book-keeping, and return the delta
  delta_->timestamp_ns = timestamp_ns;
  tracking_info_.prev_active_faces = delta_->getNumActiveFaces();
  tracking_info_.prev_active_vertices = delta_->getNumActiveVertices();
  ++tracking_info_.sequence_number;
  if (tracking_info_.sequence_number == 0) {
    // Roll-over is rare but 0 would conflict with the initial values for VertexInfo and
    // BlockInfo. Incorrect things can happen when the sequence number rolls over AND an
    // old block with the same sequence number as the current one is newly observed.
    tracking_info_.sequence_number = 1;
  }

  tracking_info_.prev_to_curr = std::make_shared<std::map<size_t, size_t>>();
  prev_archived_vertices_ = delta_->getNumArchivedVertices();

  SPARK_LOG(DEBUG) << "Finished update with " << summarizeDelta(*delta_);
  return std::move(delta_);
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

void DeltaCompression::addActiveVertices(HashedIndexMapping* remapping) {
  for (auto& [vertex_idx, info] : vertices_map_) {
    const auto new_idx = delta_->addVertex(info.pos, info.traits);
    if (info.sequence_number != tracking_info_.sequence_number) {
      // if we haven't seen this vertex in this pass, add to prev_to_curr_ map
      addIndexRemap(info.mesh_index, new_idx);
    } else {
      if (!info.is_new) {
        // we use the active remapping slot as temporary variable to cache
        // the index in the previous delta
        const size_t prev_mesh_index = active_remapping_[info.mesh_index];
        addIndexRemap(prev_mesh_index, new_idx);
      } else {
        info.is_new = false;
      }

      // set to correct delta index for face construction
      active_remapping_[info.mesh_index] = new_idx;
    }

    // set mesh index to point to the correct index in the current delta
    info.mesh_index = new_idx;
  }

  if (!remapping) {
    return;
  }

  // if we're tracking the remapping for input vertices, we need to update the active
  // indices to now point to their location in the delta
  for (auto& [idx, block_remapping] : *remapping) {
    for (auto& [source_idx, target_idx] : block_remapping) {
      target_idx = active_remapping_.at(target_idx);
    }
  }
}

void DeltaCompression::addActiveFaces() {
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
  // note that we only need to check for duplicates per each "type" of face
  RedundancyChecker checker;
  for (auto& [idx, block] : block_info_map_) {
    const bool was_updated = block.sequence_number == tracking_info_.sequence_number;
    for (auto& face : block.faces) {
      if (was_updated) {
        // face points to active vertices, so grab actual delta index and add to
        // remapping
        face[0] = active_remapping_[face[0]];
        face[1] = active_remapping_[face[1]];
        face[2] = active_remapping_[face[2]];
      } else {
        face[0] = remapIndex(face[0]);
        face[1] = remapIndex(face[1]);
        face[2] = remapIndex(face[2]);
      }

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
  // Only reset archive delta if necessary (to allow for multiple archive calls)
  if (!archive_delta_) {
    archive_delta_.reset(new MeshDelta(tracking_info_));
  }

  // Populate current archived indices
  std::unordered_set<size_t> pending_vertices;
  for (const auto& pending : archived_vertices_) {
    pending_vertices.insert(pending.mesh_index);
  }

  // Add newly archived vertices to archival candidates (and mark archived blocks)
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

  // Mark vertices that can be archived by checking pending faces for archive
  for (const auto& idx : to_erase) {
    const auto& block_info = block_info_map_[idx];
    for (const auto& face : block_info.faces) {
      markBoundaryVertices(face, pending_vertices);
    }
  }

  for (const auto& face : archived_faces_) {
    markBoundaryVertices(face, pending_vertices);
  }

  // Sweep archived vertices
  size_t boundary = archived_vertices_.size();
  for (size_t i = 0; i < boundary; ++i) {
    const auto& info = archived_vertices_[i];
    if (!pending_vertices.count(info.mesh_index)) {
      continue;
    }

    // add newly archived vertex to mesh delta
    const auto new_index = archive_delta_->addVertex(info.pos, info.traits, true);
    addIndexRemap(info.mesh_index, new_index);
    // "delete" vertex by swapping to end and decreasing size
    std::swap(archived_vertices_[i], archived_vertices_[boundary - 1]);
    boundary -= 1;
    i -= 1;
  }

  // Add boundary vertices to delta (allows archiving of faces pointing to pending or
  // archived vertices)
  archived_vertices_.resize(boundary);
  addPendingVertices(*archive_delta_);

  // Sweep archived faces
  RedundancyChecker checker;
  for (const auto& idx : to_erase) {
    archiveBlockFaces(block_info_map_.at(idx), checker);
    block_info_map_.erase(idx);
  }

  SPARK_LOG(DEBUG) << "Finished archive with " << summarizeDelta(*archive_delta_);
}

void DeltaCompression::archiveBlockFaces(const BlockInfo& block_info,
                                         RedundancyChecker& checker) {
  // prev_to_curr_ points from the previous active index to the current archival index
  // of all vertices archived from the block being archived. Any active vertex will not
  // be in the remapping
  for (const auto& face : block_info.faces) {
    const auto can_archive =
        checkFaceAll(face, [this](auto v) { return indexInRemap(v); });
    if (!can_archive) {
      // push any face touching active vertices to be archived later
      // we don't remap here as we can't tell whether an index was remapped here or not
      // when adding partially active faces into the delta
      archived_faces_.push_back(face);
      continue;
    }

    // remap face to respect newly archived vertices
    const auto f_n = applyToFace(face, [this](auto idx) { return remapIndex(idx); });
    if (!faceIsValid(f_n)) {
      continue;
    }

    if (!checker.check(f_n)) {
      continue;
    }

    checker.add(f_n);
    archive_delta_->addFace(f_n, true);
  }
}

void DeltaCompression::updateAndAddArchivedFaces() {
  // we want to touch anything that is only pointing to archived or pending vertices
  const auto archive_threshold =
      delta_->getNumArchivedVertices() + archived_vertices_.size();

  // note that we only need to check for duplicates per each "type" of face
  RedundancyChecker checker;
  auto iter = archived_faces_.begin();
  while (iter != archived_faces_.end()) {
    auto& face = *iter;
    face[0] = remapIndex(face[0]);
    face[1] = remapIndex(face[1]);
    face[2] = remapIndex(face[2]);
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
    const auto new_index = delta.addVertex(info.pos, info.traits);
    addIndexRemap(info.mesh_index, new_index);
    info.mesh_index = new_index;
  }
}

void DeltaCompression::addIndexRemap(size_t prev, size_t curr) {
  // Downstream usage cares about the remapping relative to the start of the previously
  // active vertices, so we remove the offset here when populating the remapping
  (*tracking_info_.prev_to_curr)[prev - prev_archived_vertices_] = curr;
}

bool DeltaCompression::indexInRemap(size_t index) const {
  // We maintain the remapping relative to the start of the previously active vertices,
  // so we remove the offset when checking for the presence of the remapped index
  if (index < prev_archived_vertices_) {
    return false;
  }

  return tracking_info_.prev_to_curr->count(index - prev_archived_vertices_);
}

size_t DeltaCompression::remapIndex(size_t index) const {
  // We maintain the remapping relative to the start of the previously active vertices,
  // so we remove the offset when getting the remapped index
  return tracking_info_.prev_to_curr->at(index - prev_archived_vertices_);
}

}  // namespace kimera_pgmo
