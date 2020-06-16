/**
 * @file   OctreeCompression.h
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#pragma once

#include <string>
#include <vector>

#include <ros/ros.h>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>

namespace mesher_mapper {

class OctreeCompression {
 public:
  typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> Octree;

  OctreeCompression();
  ~OctreeCompression();

  bool Initialize(const ros::NodeHandle& n,
                  double resolution,
                  std::string label);

  bool PublishVertices();

  bool PublishMesh();

  inline void getVertices(PointCloud::Ptr vertices) const {
    *vertices = *vertices_;
  }

  inline void getPolygons(std::vector<pcl::Vertices>* polygons) {
    *polygons = polygons_;
  }

  inline void getVerticesTimestamps(std::vector<double>* vertices_times) {
    *vertices_times = vertices_latest_time_;
  }

 private:
  bool LoadParameters(const ros::NodeHandle& n);
  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Callback for input mesh
  void InsertMesh(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);

  std::string frame_id_;
  PointCloud::Ptr vertices_;
  std::vector<pcl::Vertices> polygons_;
  Octree::Ptr octree_;

  std::vector<std::vector<pcl::Vertices>> adjacent_surfaces_;
  std::vector<double> vertices_latest_time_;

  double octree_resolution_;
  std::string label_;

  // Publishers
  ros::Publisher vertices_pub_;
  ros::Publisher mesh_pub_;

  ros::Subscriber mesh_sub_;
};
}  // namespace mesher_mapper