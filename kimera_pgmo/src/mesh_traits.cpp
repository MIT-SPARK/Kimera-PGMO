/**
 * @file   mesh_traits.cpp
 * @brief  adl-based mesh getters/setters and traits
 * @author Nathan Hughes
 */

#include "kimera_pgmo/mesh_traits.h"

namespace kimera_pgmo {

size_t pgmoNumVertices(const SimpleMesh& mesh) { return mesh.points.size(); }

void pgmoResizeVertices(SimpleMesh& mesh,
                        size_t size,
                        bool /* has_colors */,
                        bool /* has_stamps */,
                        bool /* has_labels */) {
  mesh.points.resize(size);
}

traits::Pos pgmoGetVertex(const SimpleMesh& mesh,
                          size_t i,
                          traits::VertexTraits* /* traits */) {
  return mesh.points.at(i);
}

void pgmoSetVertex(SimpleMesh& mesh,
                   size_t i,
                   const traits::Pos& pos,
                   const traits::VertexTraits& /* traits */) {
  mesh.points.at(i) = pos;
}

size_t pgmoNumFaces(const SimpleMesh& mesh) { return mesh.faces.size(); }

void pgmoResizeFaces(SimpleMesh& mesh, size_t size) { mesh.faces.resize(size); }

traits::Face pgmoGetFace(const SimpleMesh& mesh, size_t i) { return mesh.faces.at(i); }

void pgmoSetFace(SimpleMesh& mesh, size_t i, const traits::Face& face) {
  mesh.faces.at(i) = face;
}

size_t pgmoNumFaces(const std::vector<pcl::Vertices>& faces) { return faces.size(); }

traits::Face pgmoGetFace(const std::vector<pcl::Vertices>& faces, size_t i) {
  const auto& tri = faces.at(i);
  return {tri.vertices.at(0), tri.vertices.at(1), tri.vertices.at(2)};
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
