#pragma once
#include "kimera_pgmo/KimeraPgmoMesh.h"
#include "kimera_pgmo/MeshTraits.h"

namespace kimera_pgmo {

// TODO(lschmid): Clean this up at some point.

/**
 * @brief Fill mesh_msgs::TriangleMesh message from mesh
 * @param vertices Mesh vertices to export
 * @param faces Mesh faces to export
 */
template <typename Vertices, typename Faces>
KimeraPgmoMesh toMsg(const Vertices& vertices, const Faces& faces) {
  KimeraPgmoMesh msg;
  const auto num_vertices = traits::num_vertices(vertices);
  if (!num_vertices) {
    return msg;
  }

  // Convert vertices
  msg.vertices.resize(num_vertices);
  msg.vertex_colors.resize(num_vertices);
  constexpr float color_conversion = 1.0f / 255.0f;
  for (size_t i = 0; i < num_vertices; ++i) {
    std::optional<traits::Color> color;
    std::optional<uint8_t> alpha;
    const auto pos = traits::get_vertex(vertices, i, &color, &alpha);
    auto& p = msg.vertices[i];
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();

    // point color
    auto& c = msg.vertex_colors[i];
    if (color) {
      c.r = color->at(0) * color_conversion;
      c.g = color->at(1) * color_conversion;
      c.b = color->at(2) * color_conversion;
    } else {
      c.r = 0.0f;
      c.g = 0.0f;
      c.b = 0.0f;
    }
    c.a = alpha.value_or(255) * color_conversion;
  }

  // Convert polygons
  const auto num_faces = traits::num_faces(faces);
  msg.triangles.resize(num_faces);
  for (size_t i = 0; i < num_faces; i++) {
    auto& triangle = msg.triangles[i];
    const auto face = traits::get_face(faces, i);
    triangle.vertex_indices[0] = face[0];
    triangle.vertex_indices[1] = face[1];
    triangle.vertex_indices[2] = face[2];
  }

  return msg;
}

/**
 * @brief Fill KimeraPgmoMesh message from mesh
 * @param mesh Mesh to export
 */
template <typename Mesh>
KimeraPgmoMesh toMsg(const Mesh& mesh) {
  // dispatch meshes that implement face and vertex traits
  return toMsg(mesh, mesh);
}

}  // namespace kimera_pgmo
