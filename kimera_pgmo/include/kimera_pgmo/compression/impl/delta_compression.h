#pragma once
#include "kimera_pgmo/compression/delta_compression.h"
#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

template <typename MeshBlocksT>
MeshDelta::Ptr DeltaCompression::update(const MeshBlocksT& mesh,
                                        uint64_t timestamp_ns,
                                        HashedIndexMapping* remapping) {
  // TODO(nathan) this can maybe happen after the remapping (and we can push a bunch of
  // code back to the source file)

  // 1) Update happens independently of calls to
  // archiveBlocks and archiveBlocksByTime. We copy over the archive delta if it exists
  // instead of making a new one.
  size_t prev_num_pending = 0;
  if (archive_delta_) {
    // we need to cache the amount of pending vertices we've already copied
    prev_num_pending = archive_delta_->vertex_updates_.size() -
                       archive_delta_->getNumArchivedVertices();
    delta_ = archive_delta_;
    archive_delta_.reset();
  } else {
    delta_ = std::make_shared<MeshDelta>(tracking_info_.with_timestamp(timestamp_ns));
  }

  // 2) Compute the actual compression of the latest mesh, determining the remapping
  // between vertex indices in the provided mesh to vertices in the compressed mesh.
  updateRemapping(mesh, timestamp_ns);

  // Applies any pending vertices that may be left over from old archival passes or from
  // removed block observations
  addPendingVertices(*delta_, prev_num_pending);

  // 3) Add all active vertices and faces to the output mesh delta. This is any
  // compressed vertex that is the result of one or more blocks that hasn't
  // been archived and any face from any block that hasn't been archived.
  // note: this provides an invariant: every face that belongs to a block in
  // block_info_map_ has the correct indices after this is called and before the next
  // updateRemapping is called. This means that anyone archiving blocks can directly use
  // the faces without doing any remapping
  addActiveVertices();
  addActiveFaces(remapping);

  // 4) Update all partially archived faces with the new remapping. This remaps
  // previously partially archived faces with any newly archived vertices and also
  // archives them in the delta if they only point at archived vertices. Note that they
  // cannot be duplicates of any active faces in the new mesh delta or any fully
  // archived faces in the cached mesh delta as they must point to at least one vertex
  // that was archived before this pass that no other active face points to and must
  // also point to at least one active vertex that was archived this pass.
  updateAndAddArchivedFaces();

  // 5) Increment the pass number, finalize mesh delta book-keeping, and return the new
  // mesh update.
  // TODO(nathan) fix this
  // num_archived_vertices_ = delta_->getTotalArchivedVertices();
  // num_archived_faces_ = delta_->getTotalArchivedFaces();

  ++tracking_info_.sequence_number;
  if (tracking_info_.sequence_number == 0) {
    // NOTE(nathan) roll-over is rare but 0 would conflict with the initial values for
    // VertexInfo and BlockInfo. Incorrect things can happen when the sequence number
    // rolls over AND an old block with the same sequence number as the current one is
    // newly observed.
    tracking_info_.sequence_number = 1;
  }

  return delta_;
}

template <typename MeshBlockT>
void DeltaCompression::updateRemapping(const MeshBlockT& mesh, uint64_t stamp_ns) {
  // For every block in the latest input mesh:
  //   - initialize book-keeping for previously unobserved blocks
  //   - add all vertices from the block into the compression (this increases ref
  //   counts)
  //   - remove any previous observations from the block if the block isn't new (this
  //     decreases ref counts to be correct)
  active_remapping_.clear();
  for (const auto& [block_index, block] : mesh) {
    bool is_block_new = false;
    auto block_iter = block_info_map_.find(block_index);
    if (block_iter == block_info_map_.end()) {
      is_block_new = true;
      block_iter = block_info_map_.insert({block_index, {{}, stamp_ns, {}}}).first;
    }

    auto& block_info = block_iter->second;
    block_info.update_time_ns = stamp_ns;
    block_info.sequence_number = tracking_info_.sequence_number;
    block_info.indices.clear();

    spatial_hash::LongIndexSet curr_voxels;
    for (size_t i = 0; i < traits::num_vertices(block); ++i) {
      traits::VertexTraits traits;
      const auto pos = traits::get_vertex(block, i);
      addPoint(pos, traits, block_info.indices, curr_voxels);
    }

    if (!is_block_new) {
      removeBlockObservations(block_info.vertices);
    }

    block_info.vertices = curr_voxels;
  }
}

}  // namespace kimera_pgmo
