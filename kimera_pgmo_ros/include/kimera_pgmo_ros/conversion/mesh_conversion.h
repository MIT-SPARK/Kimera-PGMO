/**
 * @file   mesh_conversion.h
 * @brief  Conversions to and from KimeraPgmoMesh message type
 * @author Yun Chang
 */

#pragma once

#include <kimera_pgmo/mesh_traits.h>
#include <kimera_pgmo/utils/logging.h>
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kimera_pgmo::conversions {

// TODO(lschmid): Clean up interfaces here and unify with ros_conversion.

using kimera_pgmo::traits::Timestamp;

/*! \brief Convert a mesh to a pgmo mesh msg
 *  - robot_id: robot id
 *  - vertices: vertices (as point cloud) of the mesh
 *  - faces: surfaces of mesh (as pcl::Vertices polygons)
 *  - msg: output message
 *  - index_mapping: optional mapping to vertices in the deformation graph
 *  - header: optional header to use
 */
template <typename Vertices, typename Faces>
void fillMsg(size_t robot_id,
             const Vertices& vertices,
             const Faces& faces,
             kimera_pgmo_msgs::KimeraPgmoMesh& msg,
             const IndexMapping* index_mapping = nullptr,
             const std::optional<std_msgs::Header>& header = std::nullopt);

template <typename Mesh>
void fillMsg(size_t robot_id,
             const Mesh& mesh,
             kimera_pgmo_msgs::KimeraPgmoMesh& msg,
             const IndexMapping* index_mapping = nullptr,
             const std::optional<std_msgs::Header>& header = std::nullopt) {
  fillMsg(robot_id, mesh, mesh, msg, index_mapping, header);
}

template <typename Vertices, typename Faces>
kimera_pgmo_msgs::KimeraPgmoMesh::Ptr toMsg(
    size_t robot_id,
    const Vertices& vertices,
    const Faces& faces,
    const IndexMapping* index_mapping = nullptr,
    const std::optional<std_msgs::Header>& header = std::nullopt) {
  kimera_pgmo_msgs::KimeraPgmoMesh::Ptr msg(new kimera_pgmo_msgs::KimeraPgmoMesh());
  fillMsg(robot_id, vertices, faces, *msg, index_mapping, header);
  return msg;
}

template <typename Mesh>
kimera_pgmo_msgs::KimeraPgmoMesh::Ptr toMsg(
    size_t robot_id,
    const Mesh& mesh,
    const IndexMapping* index_mapping = nullptr,
    const std::optional<std_msgs::Header>& header = std::nullopt) {
  return toMsg(robot_id, mesh, mesh, index_mapping, header);
}

template <typename Vertices, typename Faces>
void fillMsg(size_t robot_id,
             const Vertices& vertices,
             const Faces& faces,
             kimera_pgmo_msgs::KimeraPgmoMesh& msg,
             const IndexMapping* index_mapping,
             const std::optional<std_msgs::Header>& header) {
  // a little inefficient, but easier than manually clearing everything
  msg = kimera_pgmo_msgs::KimeraPgmoMesh();
  msg.ns = std::to_string(robot_id);
  if (header) {
    msg.header = *header;
  }

  const auto num_vertices = traits::num_vertices(vertices);
  if (num_vertices == 0) {
    return;
  }

  // Convert vertices
  msg.vertices.reserve(num_vertices);
  msg.vertex_colors.reserve(num_vertices);
  msg.vertex_indices.reserve(num_vertices);
  msg.vertex_stamps.reserve(num_vertices);

  for (size_t i = 0; i < num_vertices; ++i) {
    traits::VertexTraits traits;
    const auto pos = traits::get_vertex(vertices, i, &traits);
    // tf2::eigen doesn't handle floats well, so we manually convert
    auto& p = msg.vertices.emplace_back();
    p.x = pos.x();
    p.y = pos.y();
    p.z = pos.z();

    if (traits.color) {
      constexpr float color_conv_factor = 1.0f / std::numeric_limits<uint8_t>::max();
      const auto& color_ref = *traits.color;
      auto& c = msg.vertex_colors.emplace_back();
      c.r = color_conv_factor * color_ref[0];
      c.g = color_conv_factor * color_ref[1];
      c.b = color_conv_factor * color_ref[2];
      c.a = color_conv_factor * color_ref[3];
    }

    if (traits.stamp) {
      auto& timestamp = msg.vertex_stamps.emplace_back();
      timestamp.fromNSec(*traits.stamp);
    }

    if (index_mapping) {
      auto& idx = msg.vertex_indices.emplace_back();
      auto iter = index_mapping->find(i);
      idx = (iter == index_mapping->end()) ? -1 : iter->second;
    }
  }

  // Convert polygons
  const auto num_faces = traits::num_faces(faces);
  msg.triangles.resize(num_faces);
  for (size_t i = 0; i < num_faces; i++) {
    const auto face = traits::get_face(faces, i);
    auto& triangle = msg.triangles[i];
    triangle.vertex_indices[0] = face[0];
    triangle.vertex_indices[1] = face[1];
    triangle.vertex_indices[2] = face[2];
  }
}

template <typename Vertices, typename Faces>
void fillFromMsg(const kimera_pgmo_msgs::KimeraPgmoMesh& msg,
                 Vertices& vertices,
                 Faces& faces,
                 std::vector<int>* graph_indices = nullptr);

template <typename Mesh>
void fillFromMsg(const kimera_pgmo_msgs::KimeraPgmoMesh& msg,
                 Mesh& mesh,
                 std::vector<int>* graph_indices = nullptr) {
  fillFromMsg(msg, mesh, mesh, graph_indices);
}

template <typename Vertices, typename Faces>
void fillFromMsg(const kimera_pgmo_msgs::KimeraPgmoMesh& msg,
                 Vertices& vertices,
                 Faces& faces,
                 std::vector<int>* graph_indices) {
  const auto num_vertices = msg.vertices.size();
  if (num_vertices == 0) {
    return;
  }

  const bool has_colors = num_vertices == msg.vertex_colors.size();
  const bool has_stamps = num_vertices == msg.vertex_stamps.size();
  const bool has_indices = num_vertices == msg.vertex_indices.size();
  if (graph_indices) {
    graph_indices->clear();
  }

  traits::resize_vertices(vertices, num_vertices);
  constexpr float color_conv_factor = 1.0f * std::numeric_limits<uint8_t>::max();
  for (size_t i = 0; i < num_vertices; i++) {
    const auto& p = msg.vertices[i];
    const traits::Pos pos(p.x, p.y, p.z);

    traits::VertexTraits traits;
    if (has_colors) {
      const std_msgs::ColorRGBA& c = msg.vertex_colors[i];
      traits.color = traits::Color{{static_cast<uint8_t>(color_conv_factor * c.r),
                                    static_cast<uint8_t>(color_conv_factor * c.g),
                                    static_cast<uint8_t>(color_conv_factor * c.b),
                                    static_cast<uint8_t>(color_conv_factor * c.a)}};
    }

    if (has_stamps) {
      traits.stamp = msg.vertex_stamps[i].toNSec();
    }

    if (has_indices && graph_indices) {
      graph_indices->push_back(msg.vertex_indices[i]);
    }

    traits::set_vertex(vertices, i, pos, traits);
  }

  const auto num_faces = msg.triangles.size();
  traits::resize_faces(faces, num_faces);
  for (size_t i = 0; i < num_faces; ++i) {
    const auto& face_vec = msg.triangles[i].vertex_indices;
    traits::Face face{face_vec[0], face_vec[1], face_vec[2]};
    traits::set_face(faces, i, face);
  }
}  // namespace kimera_pgmo::conversions

/*! \brief Convert a mesh to a pgmo mesh msg
 *  - robot_id: robot id
 *  - vertices: vertices (as point cloud) of the mesh
 *  - faces: surfaces of mesh (as pcl::Vertices polygons)
 *  - msg: output message
 *  - index_mapping: optional mapping to vertices in the deformation graph
 *  - header: optional header to use
 */
kimera_pgmo_msgs::KimeraPgmoMesh::Ptr toMsg(
    size_t robot_id,
    const pcl::PolygonMesh& mesh,
    const std::vector<Timestamp>& stamps,
    const std::string& frame_id,
    const IndexMapping* index_mapping = nullptr);

/*! \brief Convert a mesh to a pgmo mesh msg
 *  - robot_id: robot id
 *  - vertices: vertices (as point cloud) of the mesh
 *  - faces: surfaces of mesh (as pcl::Vertices polygons)
 *  - msg: output message
 *  - index_mapping: optional mapping to vertices in the deformation graph
 *  - header: optional header to use
 */
kimera_pgmo_msgs::KimeraPgmoMesh::Ptr toMsg(
    size_t robot_id,
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const std::vector<pcl::Vertices>& faces,
    const std::vector<Timestamp>& stamps,
    const std::string& frame_id,
    const IndexMapping* index_mapping = nullptr);

pcl::PolygonMesh fromMsg(const kimera_pgmo_msgs::KimeraPgmoMesh& mesh_msg,
                         std::vector<Timestamp>* vertex_stamps = nullptr,
                         std::vector<int>* vertex_graph_indices = nullptr);

}  // namespace kimera_pgmo::conversions
