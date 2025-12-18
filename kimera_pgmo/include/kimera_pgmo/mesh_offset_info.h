#pragma once
#include <limits>
#include <set>

namespace kimera_pgmo {

struct MeshOffsetInfo {
  size_t archived_vertices = 0;
  size_t prev_archived_vertices = 0;
  size_t archived_faces = 0;
  size_t pending_faces = 0;

  size_t toGlobal(size_t local_idx) const;
  size_t toLocal(size_t global_idx) const;

  struct RemapInfo {
    size_t min_index = std::numeric_limits<size_t>::max();
    size_t max_index = 0;
    bool all_archived = false;
    std::set<size_t> deleted_indices;

    void addIndex(size_t idx);
  };
};

}  // namespace kimera_pgmo
