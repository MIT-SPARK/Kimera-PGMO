/**
 * @file   vertex_update.h
 * @brief  Implementations of vertex info merging
 * @author Nathan Hughes
 */
#pragma once

#include <cstdint>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

struct DefaultVertexUpdate {
  void operator()(uint64_t timestamp_ns,
                  const traits::Pos& pos,
                  const traits::VertexTraits& traits,
                  traits::Pos& target_pos,
                  traits::VertexTraits& target_traits) const;
};

}  // namespace kimera_pgmo
