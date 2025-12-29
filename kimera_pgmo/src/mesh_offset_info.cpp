#include "kimera_pgmo/mesh_offset_info.h"

#include <stdexcept>

namespace kimera_pgmo {

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

std::optional<size_t> MeshOffsetInfo::remapGlobalVertex(const Remap& remap,
                                                        size_t index) const {
  if (index < prev_archived_vertices) {
    return index;
  }

  const auto local_idx = toLocalVertex(index);
  auto iter = remap.find(local_idx);
  if (iter == remap.end()) {
    return std::nullopt;
  }

  return toGlobalVertex(iter->second);
}

void MeshOffsetInfo::RemapInfo::addIndex(size_t idx) {
  min_index = std::min(min_index, idx);
  max_index = std::max(max_index, idx);
}

}  // namespace kimera_pgmo
