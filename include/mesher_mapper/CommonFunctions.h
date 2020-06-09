/**
 * @file   CommonFunctions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <string>

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <mesh_msgs/TriangleMesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl_msgs/PolygonMesh.h>
#include <voxblox_msgs/Mesh.h>

namespace mesher_mapper {

void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh);

void WriteMeshToPly(const std::string& filename, const pcl::PolygonMesh& mesh);

mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(
    const pcl::PolygonMesh& polygon_mesh);

pcl::PolygonMesh TriangleMeshMsgToPolygonMesh(
    const mesh_msgs::TriangleMesh& mesh_msg);

gtsam::Pose3 RosToGtsam(const geometry_msgs::Pose& transform);

pcl::PolygonMesh VoxbloxGetLastMeshBlock(
    const voxblox_msgs::Mesh::ConstPtr& voxblox_mesh);
}  // namespace mesher_mapper
