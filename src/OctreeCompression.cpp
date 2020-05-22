/**
 * @file   OctreeCompression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <ros/ros.h>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/OctreeCompression.h"

namespace mesher_mapper {

OctreeCompression::OctreeCompression() { map_data_.reset(new PointCloud); }

OctreeCompression::~OctreeCompression() {}

bool OctreeCompression::Initialize(const ros::NodeHandle& n) {
  if (!LoadParameters(n)) {
    ROS_ERROR("OctreeCompression: failed to load parameters.");
    return false;
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("OctreeCompression: failed to register callbacks.");
    return false;
  }

  ROS_INFO("Initialized Octree Compression module.");
  return true;
}

bool OctreeCompression::LoadParameters(const ros::NodeHandle& n) {
  if (!n.getParam("compression/resolution", octree_resolution_)) return false;

  if (!n.getParam("compression/frame_id", frame_id_)) return false;

  // Initialize octree
  map_octree_.reset(new Octree(octree_resolution_));
  map_octree_->setInputCloud(map_data_);

  return true;
}

bool OctreeCompression::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create local nodehandle to manage callbacks
  ros::NodeHandle nl(n);

  map_pub_ = nl.advertise<PointCloud>("octree_map", 10, true);
  mesh_sub_ =
      nl.subscribe("input_mesh", 10, &OctreeCompression::InsertMesh, this);
  return true;
}

void OctreeCompression::InsertMesh(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  // Convert polygon mesh vertices to pointcloud
  pcl::PolygonMesh polygon_mesh = TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
  PointCloud::Ptr new_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(polygon_mesh.cloud, *new_cloud);

  double min_x, min_y, min_z, max_x, max_y, max_z;

  bool is_in_box;

  for (size_t i = 0; i < new_cloud->points.size(); ++i) {
    const pcl::PointXYZ p = new_cloud->points[i];
    map_octree_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    is_in_box = (p.x >= min_x && p.x <= max_x) &&
                (p.y >= min_y && p.y <= max_y) &&
                (p.z >= min_z && p.z <= max_z);
    if (!is_in_box || !map_octree_->isVoxelOccupiedAtPoint(p)) {
      map_octree_->addPointToCloud(p, map_data_);
    }
  }

  PublishMap();
}

bool OctreeCompression::PublishMap() { map_pub_.publish(map_data_); }

}  // namespace mesher_mapper
