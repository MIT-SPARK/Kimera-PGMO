/**
 * @file   OctreeCompression.h
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#pragma once

#include <ros/ros.h>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>

namespace mesher_mapper {

class OctreeCompression {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;

  OctreeCompression();
  ~OctreeCompression();

  bool Initialize(const ros::NodeHandle& n);

  bool PublishMap();

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for input mesh
  void InsertMesh(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);

  std::string frame_id_;
  PointCloud::Ptr map_data_;
  Octree::Ptr map_octree_;

  double octree_resolution_;

  // Publishers
  ros::Publisher map_pub_;

  ros::Subscriber mesh_sub_;
};
}  // namespace mesher_mapper