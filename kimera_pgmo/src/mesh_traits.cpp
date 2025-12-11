/**
 * @file   mesh_traits.cpp
 * @brief  adl-based mesh getters/setters and traits
 * @author Nathan Hughes
 */

#include "kimera_pgmo/mesh_traits.h"

namespace kimera_pgmo {

size_t pgmoNumFaces(const std::vector<pcl::Vertices>& faces) { return faces.size(); }

traits::Face pgmoGetFace(const std::vector<pcl::Vertices>& faces, size_t i) {
  const auto& tri = faces.at(i);
  return {static_cast<size_t>(tri.vertices.at(0)),
          static_cast<size_t>(tri.vertices.at(1)),
          static_cast<size_t>(tri.vertices.at(2))};
}

void pgmoResizeFaces(std::vector<pcl::Vertices>& faces, size_t size) {
  faces.resize(size);
}

void pgmoSetFace(std::vector<pcl::Vertices>& faces,
                 size_t i,
                 const traits::Face& face) {
  auto& tri = faces.at(i);
  tri.vertices.clear();
  tri.vertices.push_back(face[0]);
  tri.vertices.push_back(face[1]);
  tri.vertices.push_back(face[2]);
}

}  // namespace kimera_pgmo
