#include "kimera_pgmo/mesh_offset_info.h"

namespace kimera_pgmo {

size_t MeshOffsetInfo::toGlobal(size_t local_idx) const {
  return local_idx + prev_archived_vertices;
}

size_t MeshOffsetInfo::toLocal(size_t global_idx) const {
  return global_idx - prev_archived_vertices;
}

void MeshOffsetInfo::RemapInfo::addIndex(size_t idx) {
  min_index = std::min(min_index, idx);
  max_index = std::max(max_index, idx);
}

}  // namespace kimera_pgmo
