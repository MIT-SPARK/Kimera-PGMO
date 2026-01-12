/**
 * @file   mesh_traits.cpp
 * @brief  adl-based mesh getters/setters and traits
 * @author Nathan Hughes
 */

#include "kimera_pgmo/mesh_traits.h"

namespace kimera_pgmo {

using traits::Face;
using traits::Pos;
using traits::Vertex;
using traits::VertexProperties;
using traits::VertexTraits;

size_t pgmoNumVertices(const std::vector<Vertex>& vertices) { return vertices.size(); }

Pos pgmoGetVertex(const std::vector<Vertex>& vertices, size_t i, VertexTraits* traits) {
  const auto& v = vertices.at(i);
  if (traits) {
    *traits = v.traits;
  }

  return v.pos;
}

VertexProperties pgmoGetVertexProperties(const std::vector<Vertex>& vertices) {
  return vertices.empty() ? VertexProperties{} : vertices.front().traits.properties;
}

void pgmoResizeVertices(std::vector<Vertex>& vertices, size_t size) {
  vertices.resize(size);
}

void pgmoSetVertex(std::vector<Vertex>& vertices,
                   size_t i,
                   const Pos& pos,
                   const VertexTraits* traits) {
  auto& v = vertices.at(i);
  v.pos = pos;
  if (traits) {
    v.traits = *traits;
  }
}

size_t pgmoNumFaces(const std::vector<Face>& faces) { return faces.size(); }

Face pgmoGetFace(const std::vector<Face>& faces, size_t i) { return faces.at(i); }

void pgmoResizeFaces(std::vector<Face>& faces, size_t size) { faces.resize(size); }

void pgmoSetFace(std::vector<Face>& faces, size_t i, const Face& face) {
  faces.at(i) = face;
}

}  // namespace kimera_pgmo
