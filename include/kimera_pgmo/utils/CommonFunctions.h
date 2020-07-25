/**
 * @file   CommonFunctions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <string>

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <mesh_msgs/TriangleMesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pose_graph_tools/PoseGraph.h>
#include <voxblox_msgs/Mesh.h>
#include <pcl/impl/point_types.hpp>

namespace kimera_pgmo {

typedef pose_graph_tools::PoseGraph::ConstPtr GraphMsgPtr;

void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh);

void WriteMeshToPly(const std::string& filename, const pcl::PolygonMesh& mesh);

mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(
    const pcl::PolygonMesh& polygon_mesh);

pcl::PolygonMesh TriangleMeshMsgToPolygonMesh(
    const mesh_msgs::TriangleMesh& mesh_msg);

gtsam::Pose3 RosToGtsam(const geometry_msgs::Pose& transform);

pcl::PolygonMesh CombineMeshes(const pcl::PolygonMesh& mesh1,
                               const pcl::PolygonMesh& mesh2,
                               bool check_duplicate_vertices = true);

pcl::PolygonMesh CombineMeshes(const pcl::PolygonMesh& mesh1,
                               const pcl::PolygonMesh& mesh2,
                               const std::vector<size_t>& indices_to_check,
                               std::vector<size_t>* vertex_indices);

pcl::PolygonMesh UpdateMesh(const pcl::PolygonMesh& mesh1,
                            const voxblox_msgs::MeshBlock& mesh_block,
                            const float& block_edge_length,
                            const std::vector<size_t>& original_indices,
                            std::vector<size_t>* updated_indices);

pcl::PolygonMesh VoxbloxMeshBlockToPolygonMesh(
    const voxblox_msgs::MeshBlock& mesh_block,
    float block_edge_length);

pcl::PolygonMesh VoxbloxToPolygonMesh(
    const voxblox_msgs::Mesh::ConstPtr& voxblox_mesh);

bool PolygonsEqual(const pcl::Vertices& p1, const pcl::Vertices& p2);

// Convert gtsam posegaph to PoseGraph msg
GraphMsgPtr GtsamGraphToRos(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values,
    const std::vector<ros::Time>& timestamps = std::vector<ros::Time>());
}  // namespace kimera_pgmo
