/**
 * @file   TriangleMeshConversion.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once
#include <mesh_msgs/TriangleMesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "kimera_pgmo/MeshTraits.h"

namespace kimera_pgmo {

/*! \brief Convert pcl PolygonMesh to mesh_msg TriangleMesh
 *  - polygon_mesh: mesh to convert
 */
mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(
    const pcl::PolygonMesh& polygon_mesh);

/*! \brief Convert a mesh represented by vertices and polygons to a mesh_msg
 * TriangleMesh
 *  - vertices: vertices (as point cloud) of the mesh
 *  - polygons: surfaces of mesh (as pcl::Vertices polygons)
 */
mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const std::vector<pcl::Vertices>& polygons);

/*! \brief Convert a mesh_msg TriangleMesh to PolygonMesh
 *  - mesh_msg: TriangleMesh mesh to be converted
 *  - outputs mesh as PolygonMesh type
 */
pcl::PolygonMesh TriangleMeshMsgToPolygonMesh(const mesh_msgs::TriangleMesh& mesh_msg);

/**
 * @brief Fill mesh_msgs::TriangleMesh message from mesh
 * @param vertices Mesh vertices to export
 * @param faces Mesh faces to export
 * @param msg Message to fill
 */
template <typename Vertices, typename Faces>
void fillTriangleMeshMsg(const Vertices& vertices,
                         const Faces& faces,
                         mesh_msgs::TriangleMesh& msg) {
  constexpr float color_conv_factor = 1.0f / std::numeric_limits<uint8_t>::max();
  const auto num_vertices = traits::num_vertices(vertices);
  if (!num_vertices) {
    return;
  }

  // Convert vertices
  msg.vertices.resize(num_vertices);
  msg.vertex_colors.resize(num_vertices);
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
      const auto& rgb = *color;
      c.r = color_conv_factor * rgb[0];
      c.g = color_conv_factor * rgb[1];
      c.b = color_conv_factor * rgb[2];
    } else {
      c.r = 0.0;
      c.g = 0.0;
      c.b = 0.0;
    }
    c.a = color_conv_factor * alpha.value_or(255);
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
}

/**
 * @brief Fill mesh_msgs::TriangleMesh message from mesh
 * @param mesh Mesh to export
 * @param msg Message to fill
 */
template <typename Mesh>
void fillTriangleMeshMsg(const Mesh& mesh, mesh_msgs::TriangleMesh& msg) {
  // dispatch meshes that implement face and vertex traits
  fillTriangleMeshMsg(mesh, mesh, msg);
}

template <typename Vertices, typename Faces>
void fillFromTriangleMeshMsg(const mesh_msgs::TriangleMesh& msg,
                             Vertices& vertices,
                             Faces& faces) {
  if (msg.vertices.size() == 0) {
    return;
  }

  bool has_color = (msg.vertex_colors.size() == msg.vertices.size());
  constexpr float color_conv_factor = 1.0f * std::numeric_limits<uint8_t>::max();
  traits::resize_vertices(vertices, msg.vertices.size());
  for (size_t i = 0; i < msg.vertices.size(); ++i) {
    const auto& p = msg.vertices[i];
    Eigen::Vector3d pos;
    pos.x() = p.x;
    pos.y() = p.y;
    pos.z() = p.z;

    std::optional<traits::Color> color;
    std::optional<uint8_t> alpha;
    if (has_color) {
      const auto& c = msg.vertex_colors[i];
      color = traits::Color{static_cast<uint8_t>(color_conv_factor * c.r),
                            static_cast<uint8_t>(color_conv_factor * c.g),
                            static_cast<uint8_t>(color_conv_factor * c.b)};
      alpha = static_cast<uint8_t>(color_conv_factor * c.a);
    }

    traits::set_vertex(vertices, i, pos.cast<float>(), color, alpha);
  }

  // Convert polygons
  traits::resize_faces(faces, msg.triangles.size());
  for (size_t i = 0; i < msg.triangles.size(); ++i) {
    const auto& tri = msg.triangles[i].vertex_indices;
    const std::array<uint64_t, 3> face{tri[0], tri[1], tri[2]};
    traits::set_face(faces, i, face);
  }
}

template <typename Mesh>
void fillFromTriangleMeshMsg(const mesh_msgs::TriangleMesh& msg, Mesh& mesh) {
  // dispatch meshes that implement face and vertex traits
  fillFromTriangleMeshMsg(msg, mesh, mesh);
}

}  // namespace kimera_pgmo
