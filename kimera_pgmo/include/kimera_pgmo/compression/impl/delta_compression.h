#pragma once
#include "kimera_pgmo/compression/delta_compression.h"
#include "kimera_pgmo/mesh_traits.h"

namespace kimera_pgmo {

template <typename MeshBlocksT, typename MergeT>
MeshDelta::Ptr DeltaCompression::update(const MeshBlocksT& mesh,
                                        uint64_t timestamp_ns,
                                        HashedIndexMapping* remapping) {
  // For every block in the latest input mesh:
  //   - initialize book-keeping for previously unobserved blocks
  //   - add all vertices from the block into the compression (increases refs)
  //   - remove previous observations from the block if not new (decreases refs)
  active_remapping_.clear();
  for (const auto& [block_index, block] : mesh) {
    bool is_block_new = false;
    auto block_iter = block_info_map_.find(block_index);
    if (block_iter == block_info_map_.end()) {
      is_block_new = true;
      block_iter = block_info_map_.insert({block_index, {timestamp_ns}}).first;
    }

    auto& block_info = block_iter->second;
    block_info.update_time_ns = timestamp_ns;
    block_info.sequence_number = tracking_info_.sequence_number;
    block_info.faces.clear();

    IndexMapping* block_remap = nullptr;
    if (remapping) {
      block_remap = &(remapping->insert({block_index, {}}).first->second);
    }

    const auto num_vertices = traits::num_vertices(block);
    spatial_hash::LongIndexSet curr_voxels;
    std::vector<size_t> local_remapping;
    local_remapping.reserve(num_vertices);
    for (size_t i = 0; i < num_vertices; ++i) {
      traits::VertexTraits traits;
      const auto pos = traits::get_vertex(block, i, &traits);
      addPoint<MergeT>(timestamp_ns, pos, traits, local_remapping, curr_voxels);
      if (block_remap) {
        block_remap->insert({i, local_remapping.back()});
      }
    }

    for (size_t i = 0; i < traits::num_faces(block); ++i) {
      const auto face = traits::get_face(block, i);
      auto& new_face = block_info.faces.emplace_back();
      new_face[0] = local_remapping.at(face[0]);
      new_face[1] = local_remapping.at(face[1]);
      new_face[2] = local_remapping.at(face[2]);
    }

    if (!is_block_new) {
      removeBlockObservations(block_info.vertices);
    }

    block_info.vertices = curr_voxels;
  }

  return computeDelta(timestamp_ns, remapping);
}

template <typename MergeT>
void DeltaCompression::addPoint(uint64_t timestamp,
                                const traits::Pos& pos,
                                const traits::VertexTraits& traits,
                                std::vector<size_t>& face_map,
                                spatial_hash::LongIndexSet& curr_voxels) {
  constexpr static const MergeT merger;

  // hash vertex at compression size to map to compressed vertex (if it exists)
  const auto vertex_index = grid_.toIndex(pos);
  auto info_iter = vertices_map_.find(vertex_index);
  if (info_iter == vertices_map_.end()) {
    // update is forced by sequence number defaulting to -1
    info_iter = vertices_map_.insert({vertex_index, {}}).first;
  }

  auto& info = info_iter->second;
  merger(timestamp, pos, traits, info);
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

}  // namespace kimera_pgmo
