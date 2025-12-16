/**
 * @file   mesh_traits.cpp
 * @brief  adl-based mesh getters/setters and traits
 * @author Nathan Hughes
 */

#include "kimera_pgmo/mesh_traits.h"

namespace kimera_pgmo {

size_t pgmoNumFaces(const std::vector<traits::Face>& faces) { return faces.size(); }

traits::Face pgmoGetFace(const std::vector<traits::Face>& faces, size_t i) {
  return faces.at(i);
}

void pgmoResizeFaces(std::vector<traits::Face>& faces, size_t size) {
  faces.resize(size);
}

void pgmoSetFace(std::vector<traits::Face>& faces, size_t i, const traits::Face& face) {
  faces.at(i) = face;
}

}  // namespace kimera_pgmo
