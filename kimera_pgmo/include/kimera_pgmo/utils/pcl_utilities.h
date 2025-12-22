/**
 * @file   pcl_utilities.h
 * @brief  Some common pcl functions used in library
 * @author Yun Chang
 */

#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <map>

namespace kimera_pgmo {

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
