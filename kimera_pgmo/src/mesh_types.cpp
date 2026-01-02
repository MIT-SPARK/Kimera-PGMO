#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo::traits {

VertexProperties& VertexProperties::operator|=(const VertexProperties& other) {
  has_color |= other.has_color;
  has_stamp |= other.has_stamp;
  has_label |= other.has_label;
  has_first_seen_stamp |= other.has_first_seen_stamp;
  return *this;
}

VertexProperties& VertexProperties::operator&=(const VertexProperties& other) {
  has_color &= other.has_color;
  has_stamp &= other.has_stamp;
  has_label &= other.has_label;
  has_first_seen_stamp &= other.has_first_seen_stamp;
  return *this;
}

bool operator==(const VertexProperties& lhs, const VertexProperties& rhs) {
  return lhs.has_color == rhs.has_color && lhs.has_stamp == rhs.has_stamp &&
         lhs.has_label == rhs.has_label &&
         lhs.has_first_seen_stamp == rhs.has_first_seen_stamp;
}

bool operator==(const VertexTraits& lhs, const VertexTraits& rhs) {
  return lhs.properties == rhs.properties && lhs.color == rhs.color &&
         lhs.stamp == rhs.stamp && lhs.label == rhs.label &&
         lhs.first_seen_stamp == rhs.first_seen_stamp;
}

}  // namespace kimera_pgmo::traits
