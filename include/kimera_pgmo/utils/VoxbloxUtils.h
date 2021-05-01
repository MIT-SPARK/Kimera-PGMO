/**
 * @file   VoxbloxUtils.h
 * @brief  Some utility functions in working with voxblox msg types
 * @author Yun Chang
 */

#pragma once

#include <boost/shared_ptr.hpp>
#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_msgs/PolygonMesh.h>
#include <voxblox_msgs/Mesh.h>

// Define BlockIndex as used in voxblox
typedef Eigen::Matrix<int, 3, 1> BlockIndex;
namespace std {
template <>
struct less<BlockIndex> {
  bool operator()(const BlockIndex& a, const BlockIndex& b) const {
    return std::lexicographical_compare(
        a.data(), a.data() + a.size(), b.data(), b.data() + b.size());
  }
};
}  // namespace std

namespace kimera_pgmo {

/*! \brief Update a polygon mesh from a voxblox mesh block
 *  outputs the polygon mesh corresponding to mesh block
 *  - mesh_block: voxblox mesh block to update mesh with
 *  - block_edge_length: block_edge_length as given in voxblox msg
 *  - vertices: the vertices of the mesh to be updated
 *  - triangles: the connections of the mesh to be updated
 *  - original_indices: if mesh block previously seen ~ the idx vertices last
 * seen
 *  - updated_indices: the idx of the points corresponding to the vertices of
 * the part of the mesh of the mesh block
 */
pcl::PolygonMesh UpdateMeshFromVoxbloxMeshBlock(
    const voxblox_msgs::MeshBlock& mesh_block,
    const float& block_edge_length,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices,
    boost::shared_ptr<std::vector<pcl::Vertices> > triangles,
    const std::vector<size_t>& original_indices,
    boost::shared_ptr<std::vector<size_t> > updated_indices,
    boost::shared_ptr<std::map<size_t, std::vector<pcl::Vertices> > >
        adjacent_surfaces);

/*! \brief When adding new mesh surface, check first if exist with a stored
 * index to adjacent surface map, and also update map if surface is new
 *  - new_triangle: new surface to be inserted
 *  - adjacent_surfaces: vertex to adjacent surfaces map
 *  returns true if surface exists
 */
bool CheckAndUpdateAdjacentSurfaces(
    const pcl::Vertices& new_triangle,
    boost::shared_ptr<std::map<size_t, std::vector<pcl::Vertices> > >
        adjacent_surfaces);

/*! \brief Convert a voxblox mesh block to a polygon mesh type
 *  - mesh_block: voxblox mesh block input
 *  - block_edge_length: block_edge_length as given in voxblox msg
 */
pcl::PolygonMesh VoxbloxMeshBlockToPolygonMesh(
    const voxblox_msgs::MeshBlock& mesh_block,
    float block_edge_length);

/*! \brief Convert a voxblox mesh block to a polygon
 * mesh (represented as pointcloud of vertices + sets of triangles )
 *  - mesh_block: voxblox mesh block input
 *  - vertices: pointcloud of generated vertices
 *  - triangles: vector of triplet indices inidicating the connections
 *  - check_duplicates_full: check for duplicated vertices against all vertices
 * (including the points already in vertices)
 */
void VoxbloxMeshBlockToPolygonMesh(
    const voxblox_msgs::MeshBlock& mesh_block,
    float block_edge_length,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices,
    boost::shared_ptr<std::vector<pcl::Vertices> > triangles,
    const bool& check_duplicates_full = false);

/*! \brief Convert a voxblox mesh ~ consisted of many mesh blocks, to a polygon
 * mesh
 *  - mesh: voxblox mesh msg input
 */
pcl::PolygonMesh VoxbloxToPolygonMesh(
    const voxblox_msgs::Mesh::ConstPtr& voxblox_mesh);

}  // namespace kimera_pgmo