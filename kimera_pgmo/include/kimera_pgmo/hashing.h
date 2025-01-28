#pragma once

#include <spatial_hash/hash.h>

#include <unordered_set>
#include <vector>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

using spatial_hash::BlockIndex;
using spatial_hash::BlockIndices;
using spatial_hash::LongIndex;
using spatial_hash::LongIndexSet;
using LongIndices = std::vector<LongIndex>;
using BlockIndexSet = std::unordered_set<BlockIndex, spatial_hash::IndexHash>;
template <typename ValueT>
using BlockIndexMap = spatial_hash::IndexHashMap<ValueT>;
template <typename ValueT>
using LongIndexMap = spatial_hash::LongIndexHashMap<ValueT>;

using HashedIndexMapping = spatial_hash::IndexHashMap<IndexMapping>;

}  // namespace kimera_pgmo
