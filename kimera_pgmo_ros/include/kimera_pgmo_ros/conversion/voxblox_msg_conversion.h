/**
 * @file   VoxbloxMsgConverion.h
 * @brief  Conversions to and from voxblox messages
 * @author Yun Chang
 * @author Nathan Hughes
 */

// TODO(lschmid): These are not used anywhere but potentially don't hurt to carry around?
#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox_msgs/Mesh.h>

#include <string>

namespace kimera_pgmo::conversions {

/*! \brief Extract point in pcl format from voxblox mesh block
 *  - mesh_block: voxblox mesh block to update mesh with
 *  - block_edge_length: block_edge_length as given in voxblox msg
 *  - vertices: the vertices of the mesh to be updated
 *  - idx: index of point to be extracted
 */
pcl::PointXYZRGBA ExtractPoint(const voxblox_msgs::MeshBlock& mesh_block,
                               const float& block_edge_length,
                               const size_t& idx);

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
    std::shared_ptr<std::vector<pcl::Vertices>> triangles,
    const std::vector<size_t>& original_indices,
    std::shared_ptr<std::vector<size_t>> updated_indices,
    std::shared_ptr<std::map<size_t, std::vector<pcl::Vertices>>> adjacent_surfaces);

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
 *  - msg_vertex_map: mapping from msg (mesh block) index to vertex index
 *  - triangles: vector of triplet indices inidicating the connections
 *  - check_duplicates_full: check for duplicated vertices against all vertices
 * (including the points already in vertices)
 */
void VoxbloxMeshBlockToPolygonMesh(
    const voxblox_msgs::MeshBlock& mesh_block,
    float block_edge_length,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices,
    std::shared_ptr<std::map<size_t, size_t>> msg_vertex_map,
    std::shared_ptr<std::vector<pcl::Vertices>> triangles,
    bool check_duplicates_full = false);

/*! \brief Convert a voxblox mesh ~ consisted of many mesh blocks, to a polygon
 * mesh
 *  - mesh: voxblox mesh msg input
 */
pcl::PolygonMesh VoxbloxToPolygonMesh(const voxblox_msgs::Mesh::ConstPtr& voxblox_mesh);

}  // namespace kimera_pgmo::conversions
