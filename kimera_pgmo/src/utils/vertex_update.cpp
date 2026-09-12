/**
 * @file   vertex_update.cpp
 * @brief  Implementations of vertex info merging
 * @author Nathan Hughes
 */
#include "kimera_pgmo/utils/vertex_update.h"

namespace kimera_pgmo {

void DefaultVertexUpdate::operator()(uint64_t timestamp_ns,
                                     const traits::Pos& pos,
                                     const traits::VertexTraits& traits,
                                     traits::Pos& target_pos,
                                     traits::VertexTraits& target_traits) const {
  target_pos = pos;
  target_traits.properties |= traits.properties;
  target_traits.properties.has_stamp = true;
  target_traits.properties.has_first_seen_stamp = true;
  target_traits.stamp = std::max(
      target_traits.stamp, traits.properties.has_stamp ? traits.stamp : timestamp_ns);
  target_traits.first_seen_stamp = std::min(
      target_traits.first_seen_stamp,
      traits.properties.has_first_seen_stamp ? traits.first_seen_stamp : timestamp_ns);

  if (traits.properties.has_color) {
    target_traits.color = traits.color;
  }

  if (traits.properties.has_label) {
    target_traits.label = traits.label;
  }
}

}  // namespace kimera_pgmo
