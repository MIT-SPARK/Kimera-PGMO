/**
 * @file   common_functions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>
#include <string>

namespace kimera_pgmo {

const std::map<size_t, char> robot_id_to_prefix = {
    {0, 'a'},
    {1, 'b'},
    {2, 'c'},
    {3, 'd'},
    {4, 'e'},
    {5, 'f'},
    {6, 'g'},
    {7, 'h'},
};

const std::map<char, size_t> robot_prefix_to_id = {
    {'a', 0},
    {'b', 1},
    {'c', 2},
    {'d', 3},
    {'e', 4},
    {'f', 5},
    {'g', 6},
    {'h', 7},
};

const std::map<size_t, char> robot_id_to_vertex_prefix = {
    {0, 's'},
    {1, 't'},
    {2, 'u'},
    {3, 'v'},
    {4, 'w'},
    {5, 'x'},
    {6, 'y'},
    {7, 'z'},
};

const std::map<char, size_t> vertex_prefix_to_id = {
    {'s', 0},
    {'t', 1},
    {'u', 2},
    {'v', 3},
    {'w', 4},
    {'x', 5},
    {'y', 6},
    {'z', 7},
};

/*! \brief Get gtsam prefix for a robot id
 *  - robot_id: id of robot from pose graph messages
 */
inline char GetRobotPrefix(size_t robot_id) {
  if (!robot_id_to_prefix.count(robot_id)) {
    return '\0';
  }
  return robot_id_to_prefix.at(robot_id);
}

/*! \brief Get gtsam prefix (for mesh vertices in deformation graph) for a robot
 * id
 *  - robot_id: id of robot from pose graph messages
 */
inline char GetVertexPrefix(size_t robot_id) {
  if (!robot_id_to_vertex_prefix.count(robot_id)) {
    return '\0';
  }
  return robot_id_to_vertex_prefix.at(robot_id);
}

/*! \brief Converts a pcl point to gtsam point3
 *  - point: pcl point
 *  - outputs gtsam point3 position
 */
template <class point_type>
inline gtsam::Point3 PclToGtsam(const point_type& point) {
  return gtsam::Point3(static_cast<double>(point.x),
                       static_cast<double>(point.y),
                       static_cast<double>(point.z));
}

/*! \brief Converts a gtsam point3 to a pcl point
 *  - point: gtsam point3
 *  - outputs pcl point
 */
template <class point_type>
inline point_type GtsamToPcl(const gtsam::Point3& point) {
  return point_type(static_cast<double>(point.x()),
                    static_cast<double>(point.y()),
                    static_cast<double>(point.z()));
}

/*! \brief Combine two meshes into one
 *  - mesh1: partial mesh represented as pcl PolygonMesh
 *  - mesh2: partial mesh represented as pcl PolygonMesh
 *  - check_duplicate_vertices: boolean to check if vertices are duplicated or
 * not
 *  - outputs combined mesh as pcl PolygonMesh
 */
pcl::PolygonMesh CombineMeshes(const pcl::PolygonMesh& mesh1,
                               const pcl::PolygonMesh& mesh2,
                               bool check_duplicate_vertices = true);

/*! \brief Combine two meshes into one
 *  - mesh1: artial mesh represented as pcl PolygonMesh
 *  - mesh2: partial mesh represented as pcl PolygonMesh
 *  - indices_to_check: check mesh2 vertices against vertices at these indices
 * in mesh1 for duplication
 *  - vertex_indices: indices of the mesh2 vertices in the combined mesh
 *  - outputs combined mesh as pcl PolygonMesh
 */
pcl::PolygonMesh CombineMeshes(const pcl::PolygonMesh& mesh1,
                               const pcl::PolygonMesh& mesh2,
                               const std::vector<size_t>& indices_to_check,
                               std::vector<size_t>* vertex_indices);

/*! \brief Append vertices and faces of one mesh to another
 *  - mesh_vertices: vertices to append to
 *  - mesh_faces: mesh faces to append to
 *  - vertices_to_add: vertices to append
 *  - faces_to_add: faces to add
 */
void AppendMesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mesh_vertices,
                std::shared_ptr<std::vector<pcl::Vertices>> mesh_faces,
                const pcl::PointCloud<pcl::PointXYZRGBA>& vertices_to_add,
                const std::vector<pcl::Vertices>& faces_to_add);

/*! \brief Check if two polygons are equal to each other
 *  - p1: polygon represented as pcl Vertices
 *  - p2: polygon represented as pcl Vertices
 */
bool PolygonsEqual(const pcl::Vertices& p1, const pcl::Vertices& p2);

/*! \brief Check if a surface exist based on previous tracked adjacent surfaces
 *  - new_surface: new surface to be inserted
 *  - adjacent_surfaces: vertex to adjacent surfaces (should correspond exactly
 * to order of vertices)
 *  returns true if surface exists
 */
bool SurfaceExists(const pcl::Vertices& new_surface,
                   const std::map<size_t, std::vector<size_t>>& adjacent_surfaces,
                   const std::vector<pcl::Vertices>& surfaces);

/*! \brief When adding new mesh surface, check first if exist with a stored
 * index to adjacent surface map, and also update map if surface is new
 *  - new_triangle: new surface to be inserted
 *  - adjacent_surfaces: vertex to adjacent surfaces map
 *  returns true if surface exists
 */
bool CheckAndUpdateAdjacentSurfaces(
    const pcl::Vertices& new_triangle,
    std::shared_ptr<std::map<size_t, std::vector<pcl::Vertices>>> adjacent_surfaces);

}  // namespace kimera_pgmo
