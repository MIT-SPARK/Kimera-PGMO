/**
 * @file   VoxbloxUtils.h
 * @brief  Some utility functions in working with voxblox msg types
 * @author Yun Chang
 */

#pragma once

#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl_msgs/PolygonMesh.h>
#include <voxblox_msgs/Mesh.h>
#include <pcl/impl/point_types.hpp>

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
 *  - mesh: mesh to be updated
 *  - mesh_block: voxblox mesh block to update mesh with
 *  - block_edge_length: block_edge_length as given in voxblox msg
 *  - original_indices: if mesh block previously seen ~ the idx vertices last
 * seen
 *  - updated_indices: the idx of the points corresponding to the vertices of
 * the part of the mesh of the mesh block
 */
pcl::PolygonMesh UpdateMeshFromVoxbloxMeshBlock(
    const pcl::PolygonMesh& mesh,
    const voxblox_msgs::MeshBlock& mesh_block,
    const float& block_edge_length,
    const std::vector<size_t>& original_indices,
    std::vector<size_t>* updated_indices);

/*! \brief Convert a voxblox mesh block to a polygon mesh type
 *  - mesh_block: voxblox mesh block input
 *  - block_edge_length: block_edge_length as given in voxblox msg
 */
pcl::PolygonMesh VoxbloxMeshBlockToPolygonMesh(
    const voxblox_msgs::MeshBlock& mesh_block,
    float block_edge_length);

/*! \brief Convert a voxblox mesh ~ consisted of many mesh blocks, to a polygon
 * mesh
 *  - mesh: voxblox mesh msg input
 */
pcl::PolygonMesh VoxbloxToPolygonMesh(
    const voxblox_msgs::Mesh::ConstPtr& voxblox_mesh);

}  // namespace kimera_pgmo