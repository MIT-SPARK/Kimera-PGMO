/**
 * @file   mesh_conversion.h
 * @brief  Conversions to and from Mesh message type
 * @author Yun Chang
 */
#pragma once

#include <kimera_pgmo/mesh_traits.h>
#include <kimera_pgmo/utils/logging.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <kimera_pgmo_msgs/msg/mesh.hpp>

#include "kimera_pgmo_ros/conversion/mesh_types.h"

namespace kimera_pgmo::conversions {

using MeshMsg = kimera_pgmo_msgs::msg::Mesh;
using OptHeader = std::optional<std_msgs::msg::Header>;
// TODO(lschmid): Clean up interfaces here and unify with ros_conversion.

/*! \brief Convert a mesh to a pgmo mesh msg
 *  - robot_id: robot id
 *  - vertices: vertices (as point cloud) of the mesh
 *  - faces: surfaces of mesh (as pcl::Vertices polygons)
 *  - msg: output message
 *  - header: optional header to use
 */
template <typename Vertices, typename Faces>
void fillMsg(size_t robot_id,
             const Vertices& vertices,
             const Faces& faces,
             MeshMsg& msg,
             const OptHeader& header = std::nullopt);

template <typename Mesh>
void fillMsg(size_t robot_id,
             const Mesh& mesh,
             MeshMsg& msg,
             const OptHeader& header = std::nullopt) {
  fillMsg(robot_id, mesh, mesh, msg, header);
}

template <typename Vertices, typename Faces>
MeshMsg::UniquePtr toMsg(size_t robot_id,
                         const Vertices& vertices,
                         const Faces& faces,
                         const OptHeader& header = std::nullopt) {
  auto msg = std::make_unique<MeshMsg>();
  fillMsg(robot_id, vertices, faces, *msg, header);
  return msg;
}

template <typename Mesh>
MeshMsg::UniquePtr toMsg(size_t robot_id,
                         const Mesh& mesh,
                         const OptHeader& header = std::nullopt) {
  return toMsg(robot_id, mesh, mesh, header);
}

template <typename Vertices, typename Faces>
void fillMsg(size_t robot_id,
             const Vertices& vertices,
             const Faces& faces,
             kimera_pgmo_msgs::msg::Mesh& msg,
             const std::optional<std_msgs::msg::Header>& header) {
  // a little inefficient, but easier than manually clearing everything
  msg = kimera_pgmo_msgs::msg::Mesh();
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
  for (size_t i = 0; i < num_vertices; ++i) {
    traits::Vertex v;
    v.pos = traits::get_vertex(vertices, i, &v.traits);
    to_ros(v, msg.vertices.emplace_back());
  }

  // Convert polygons
  const auto num_faces = traits::num_faces(faces);
  msg.triangles.resize(num_faces);
  for (size_t i = 0; i < num_faces; i++) {
    const auto face = traits::get_face(faces, i);
    to_ros(face, msg.triangles[i]);
  }
}

template <typename Vertices, typename Faces>
void fillFromMsg(const kimera_pgmo_msgs::msg::Mesh& msg,
                 Vertices& vertices,
                 Faces& faces);

template <typename Mesh>
void fillFromMsg(const kimera_pgmo_msgs::msg::Mesh& msg, Mesh& mesh) {
  fillFromMsg(msg, mesh, mesh);
}

template <typename Vertices, typename Faces>
void fillFromMsg(const kimera_pgmo_msgs::msg::Mesh& msg,
                 Vertices& vertices,
                 Faces& faces) {
  const auto num_vertices = msg.vertices.size();
  if (num_vertices == 0) {
    return;
  }

  traits::resize_vertices(vertices, num_vertices);
  for (size_t i = 0; i < num_vertices; i++) {
    const auto v = from_ros(msg.vertices[i]);
    traits::set_vertex(vertices, i, v.pos, &v.traits);
  }

  const auto num_faces = msg.triangles.size();
  traits::resize_faces(faces, num_faces);
  for (size_t i = 0; i < num_faces; ++i) {
    traits::set_face(faces, i, from_ros(msg.triangles[i]));
  }
}

/*! \brief Convert a mesh to a pgmo mesh msg
 *  - robot_id: robot id
 *  - vertices: vertices (as point cloud) of the mesh
 *  - faces: surfaces of mesh (as pcl::Vertices polygons)
 *  - msg: output message
 *  - header: optional header to use
 */
MeshMsg::UniquePtr toMsg(size_t robot_id,
                         const pcl::PolygonMesh& mesh,
                         const std::vector<traits::Timestamp>& stamps,
                         const std::string& frame_id);

/*! \brief Convert a mesh to a pgmo mesh msg
 *  - robot_id: robot id
 *  - vertices: vertices (as point cloud) of the mesh
 *  - faces: surfaces of mesh (as pcl::Vertices polygons)
 *  - msg: output message
 *  - index_mapping: optional mapping to vertices in the deformation graph
 *  - header: optional header to use
 */
MeshMsg::UniquePtr toMsg(size_t robot_id,
                         const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                         const std::vector<pcl::Vertices>& faces,
                         const std::vector<traits::Timestamp>& stamps,
                         const std::string& frame_id);

pcl::PolygonMesh fromMsg(const kimera_pgmo_msgs::msg::Mesh& mesh_msg,
                         std::vector<traits::Timestamp>* vertex_stamps = nullptr);

}  // namespace kimera_pgmo::conversions
