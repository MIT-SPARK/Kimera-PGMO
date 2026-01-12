#include "kimera_pgmo/mesh_offset_info.h"

#include <stdexcept>

namespace kimera_pgmo {

using IndexVec = std::vector<size_t>;

void MeshOffsetInfo::RemapStats::addIndex(size_t idx) {
  min_index = std::min(min_index, idx);
  max_index = std::max(max_index, idx);
}

MeshOffsetInfo::MeshOffsetInfo(size_t archived_vertices,
                               size_t prev_archived_vertices,
                               size_t archived_faces,
                               const std::shared_ptr<const Remap>& prev_to_curr)
    : archived_vertices(archived_vertices),
      prev_archived_vertices(prev_archived_vertices),
      archived_faces(archived_faces),
      prev_to_curr_(prev_to_curr) {}

size_t MeshOffsetInfo::toGlobalVertex(size_t local_idx) const {
  return local_idx + prev_archived_vertices;
}

size_t MeshOffsetInfo::toLocalVertex(size_t global_idx) const {
  if (global_idx < prev_archived_vertices) {
    throw std::logic_error("global index " + std::to_string(global_idx) +
                           " does not map to local delta index");
  }

  return global_idx - prev_archived_vertices;
}

std::optional<size_t> MeshOffsetInfo::remapGlobalVertex(size_t index) const {
  if (!prev_to_curr_) {
    throw std::logic_error("no remapping set when remapping " + std::to_string(index));
  }

  if (index < prev_archived_vertices) {
    return index;
  }

  const auto local_idx = toLocalVertex(index);
  auto iter = prev_to_curr_->find(local_idx);
  if (iter == prev_to_curr_->end()) {
    return std::nullopt;
  }

  return toGlobalVertex(iter->second);
}

void MeshOffsetInfo::remapVertexIndices(std::list<size_t>& indices,
                                        RemapStats* info) const {
  size_t index = 0;
  auto iter = indices.begin();
  while (iter != indices.end()) {
    const auto remapped = remapGlobalVertex(*iter);
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
    info->all_archived = info->max_index < archived_vertices;
  }
}

IndexVec MeshOffsetInfo::remapVertexIndices(const IndexVec& indices,
                                            RemapStats* info) const {
  size_t index = 0;
  IndexVec to_return;
  for (const auto global_idx : indices) {
    const auto remapped = remapGlobalVertex(global_idx);
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
    info->all_archived = info->max_index < archived_vertices;
  }

  return to_return;
}

}  // namespace kimera_pgmo
