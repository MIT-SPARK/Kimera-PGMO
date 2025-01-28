#pragma once
#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo_msgs/KimeraPgmoMesh.h"

namespace kimera_pgmo::conversions {

/**
 * @brief Fill mesh_msgs::TriangleMesh message from mesh
 * @param vertices Mesh vertices to export
 * @param faces Mesh faces to export
 * @param vertex_indices Optional vertex indices to export
 */
template <typename Vertices, typename Faces>
kimera_pgmo_msgs::KimeraPgmoMesh toMsg(const Vertices& vertices,
                                       const Faces& faces,
                                       std::vector<int>* vertex_indices = nullptr) {
  kimera_pgmo_msgs::KimeraPgmoMesh msg;
  const auto num_vertices = traits::num_vertices(vertices);
  if (!num_vertices) {
    return msg;
  }

  // Convert vertices
  msg.vertices.resize(num_vertices);
  msg.vertex_colors.resize(num_vertices);
  msg.vertex_stamps.reserve(num_vertices);
  constexpr float color_conversion = 1.0f / 255.0f;
  for (size_t i = 0; i < num_vertices; ++i) {
    traits::VertexTraits traits;
    const auto pos = traits::get_vertex(vertices, i, &traits);
    auto& p = msg.vertices[i];
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();

    // Optional point color
    auto& c = msg.vertex_colors[i];
    if (traits.color) {
      c.r = traits.color->at(0) * color_conversion;
      c.g = traits.color->at(1) * color_conversion;
      c.b = traits.color->at(2) * color_conversion;
      c.a = traits.color->at(3) * color_conversion;
    } else {
      c.r = 0.0f;
      c.g = 0.0f;
      c.b = 0.0f;
      c.a = 1.0f;
    }

    // Optional point stamp
    if (traits.stamp) {
      msg.vertex_stamps.emplace_back().fromNSec(*traits.stamp);
    }
  }

  // Vertex indices
  if (vertex_indices) {
    msg.vertex_indices = *vertex_indices;
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
 * @param vertex_indices Optional vertex indices to export
 */
template <typename Mesh>
kimera_pgmo_msgs::KimeraPgmoMesh toMsg(const Mesh& mesh,
                                       std::vector<int>* vertex_indices = nullptr) {
  // dispatch meshes that implement face and vertex traits
  return toMsg(mesh, mesh, vertex_indices);
}

/**
 * @brief Fill mesh from kimera_pgmo_msgs::KimeraPgmoMesh message
 * @param msg Mesh message to import
 * @param vertices Mesh vertices to fill
 * @param faces Mesh faces to fill
 * @param vertex_indices Optional vertex indices to fill
 */
template <typename Vertices, typename Faces>
void fromMsg(const kimera_pgmo_msgs::KimeraPgmoMesh& msg,
             Vertices& vertices,
             Faces& faces,
             std::vector<int>* vertex_indices = nullptr) {
  const auto num_vertices = msg.vertices.size();
  if (!num_vertices) {
    return;
  }

  // Set vertices
  traits::resize_vertices(vertices, num_vertices);
  for (size_t i = 0; i < num_vertices; i++) {
    const auto& p = msg.vertices[i];
    const traits::Pos pos(p.x, p.y, p.z);

    traits::VertexTraits traits;
    if (msg.vertex_colors.size() > i) {
      const auto& c = msg.vertex_colors[i];
      traits.color = traits::Color{static_cast<uint8_t>(c.r * 255.0f),
                                   static_cast<uint8_t>(c.g * 255.0f),
                                   static_cast<uint8_t>(c.b * 255.0f),
                                   static_cast<uint8_t>(c.a * 255.0f)};
    }
    if (msg.vertex_stamps.size() > i) {
      traits.stamp = msg.vertex_stamps[i].toNSec();
    }

    traits::set_vertex(vertices, i, pos, traits);
  }

  // Set faces.
  const auto num_faces = msg.triangles.size();
  traits::resize_faces(faces, num_faces);
  for (size_t i = 0; i < num_faces; ++i) {
    const auto& triangle = msg.triangles[i];
    traits::Face face{triangle.vertex_indices[0],
                      triangle.vertex_indices[1],
                      triangle.vertex_indices[2]};
    traits::set_face(faces, i, face);

    // Optional vertex indices
    if (vertex_indices) {
      *vertex_indices = msg.vertex_indices;
    }
  }
}

/**
 * @brief Fill mesh from KimeraPgmoMesh message
 * @param msg Mesh message to import
 * @param mesh Mesh to fill
 * @param vertex_indices Optional vertex indices to fill
 */
template <typename Mesh>
void fromMsg(const kimera_pgmo_msgs::KimeraPgmoMesh& msg,
             Mesh& mesh,
             std::vector<int>* vertex_indices = nullptr) {
  // dispatch meshes that implement face and vertex traits
  fromMsg(msg, mesh, mesh, vertex_indices);
}

}  // namespace kimera_pgmo::conversions
