#include "kimera_pgmo/pcl_mesh_traits.h"

namespace kimera_pgmo::traits {

template <>
size_t num_faces(const std::vector<pcl::Vertices>& faces) {
  return faces.size();
}

template <>
Face get_face(const std::vector<pcl::Vertices>& faces, size_t i) {
  const auto& tri = faces.at(i);
  return {static_cast<size_t>(tri.vertices.at(0)),
          static_cast<size_t>(tri.vertices.at(1)),
          static_cast<size_t>(tri.vertices.at(2))};
}

template <>
void resize_faces(std::vector<pcl::Vertices>& faces, size_t size) {
  faces.resize(size);
}

template <>
void set_face(std::vector<pcl::Vertices>& faces, size_t i, const Face& face) {
  auto& tri = faces.at(i);
  tri.vertices.clear();
  tri.vertices.push_back(face[0]);
  tri.vertices.push_back(face[1]);
  tri.vertices.push_back(face[2]);
}

}  // namespace kimera_pgmo::traits
