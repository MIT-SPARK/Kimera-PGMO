/**
 * @file   VoxbloxProcessing.h
 * @brief  VoxbloxProcessing class: process incoming voxblox meshes
 * @author Yun Chang
 */
#pragma once

#include <ros/ros.h>
#include <map>
#include <queue>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/VoxbloxUtils.h"

namespace kimera_pgmo {

class VoxbloxProcessing {
  friend class VoxbloxProcessingTest;

 public:
  VoxbloxProcessing();
  ~VoxbloxProcessing();

  // Initialize parameters, publishers, and subscribers
  bool initialize(const ros::NodeHandle& n);

 protected:
  bool loadParameters(const ros::NodeHandle& n);

  bool createPublishers(const ros::NodeHandle& n);

  bool registerCallbacks(const ros::NodeHandle& n);

  void voxbloxCallback(const voxblox_msgs::Mesh::ConstPtr& msg);

  // Creates partial mesh while updating the full mesh and also the last
  // detected mesh blocks
  pcl::PolygonMesh processVoxbloxMesh(const voxblox_msgs::Mesh::ConstPtr& msg);

  void pruneStoredBlocks(const ros::Time& latest_time);

  void publishPartialMesh(const pcl::PolygonMesh& mesh,
                          const ros::Time& stamp) const;

  void publishFullMesh(const ros::Time& stamp) const;

  // Class arguments
  ros::Subscriber voxblox_sub_;
  ros::Publisher full_mesh_pub_;
  ros::Publisher partial_mesh_pub_;

  double time_horizon_;  // only merge meshes for the blocks detected
                         // within defined time horizon (secs)

  OctreeCompressionPtr compression_;  // Allow compression of full mesh

  // Vertices of full mesh
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices_;
  // Triangles (connections) of full mesh
  std::vector<pcl::Vertices> triangles_;
};

}  // namespace kimera_pgmo
