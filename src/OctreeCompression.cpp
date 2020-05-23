/**
 * @file   OctreeCompression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <algorithm>
#include <map>

#include <ros/ros.h>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/OctreeCompression.h"

namespace mesher_mapper {

OctreeCompression::OctreeCompression() { vertices_.reset(new PointCloud); }

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
  octree_.reset(new Octree(octree_resolution_));
  octree_->setInputCloud(vertices_);

  return true;
}

bool OctreeCompression::RegisterCallbacks(const ros::NodeHandle& n) {
  // Create local nodehandle to manage callbacks
  ros::NodeHandle nl(n);

  vertices_pub_ = nl.advertise<PointCloud>("vertices", 10, true);
  mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("compressed_mesh", 10, true);
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
  std::map<size_t, size_t> remapping;

  for (size_t i = 0; i < new_cloud->points.size(); ++i) {
    const pcl::PointXYZ p = new_cloud->points[i];
    octree_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    is_in_box = (p.x >= min_x && p.x <= max_x) &&
                (p.y >= min_y && p.y <= max_y) &&
                (p.z >= min_z && p.z <= max_z);
    if (!is_in_box || !octree_->isVoxelOccupiedAtPoint(p)) {
      octree_->addPointToCloud(p, vertices_);
      remapping[i] = vertices_->points.size() - 1;
    } else {
      float unused = 0.f;
      int result_idx;
      octree_->approxNearestSearch(p, result_idx, unused);
      // Add remapping
      remapping[i] = result_idx;
    }
  }

  // Insert polygons
  for (pcl::Vertices polygon : polygon_mesh.polygons) {
    pcl::Vertices new_polygon;
    size_t largest_idx = 0;
    for (size_t idx : polygon.vertices) {
      new_polygon.vertices.push_back(remapping[idx]);
      if (remapping[idx] > largest_idx) largest_idx = remapping[idx];
    }
    // Check if this polygon already inserted
    bool surface_exist = false;
    if (largest_idx < adjacent_surfaces_.size()) {
      // Check if an existing surface
      // Iterate through adjacent surfaces of first vertex
      for (pcl::Vertices existed_p :
           adjacent_surfaces_[new_polygon.vertices[0]]) {
        if (surface_exist) break;
        // Sort before checking if vertices identical
        std::vector<uint> vertices = existed_p.vertices;
        std::sort(vertices.begin(), vertices.end());
        std::vector<uint> new_vertices = new_polygon.vertices;
        std::sort(new_vertices.begin(), new_vertices.end());
        // Check if have same number of vertices
        if (vertices.size() == new_vertices.size()) {
          for (size_t i = 0; i < vertices.size(); i++) {
            if (vertices[i] == new_vertices[i]) {
              surface_exist = false;
              break;
            }
          }
        }
      }
    }

    // If surface does not exit, add
    if (!surface_exist) {
      // Definitely a new surface
      polygons_.push_back(new_polygon);
      // Push to adjacent surfaces
      for (size_t idx : new_polygon.vertices) {
        if (idx >= adjacent_surfaces_.size()) {
          adjacent_surfaces_.push_back(std::vector<pcl::Vertices>{new_polygon});
        } else {
          adjacent_surfaces_[idx].push_back(new_polygon);
        }
      }
    }
  }
  // Publish
  if (vertices_pub_.getNumSubscribers() > 0) PublishVertices();
  if (mesh_pub_.getNumSubscribers() > 0) PublishMesh();
}

bool OctreeCompression::PublishVertices() {
  vertices_->header.frame_id = frame_id_;
  vertices_pub_.publish(vertices_);
  return true;
}

bool OctreeCompression::PublishMesh() {
  pcl::PolygonMesh new_mesh;
  pcl::toPCLPointCloud2(*vertices_, new_mesh.cloud);
  new_mesh.polygons = polygons_;
  mesh_msgs::TriangleMesh mesh_msg = PolygonMeshToTriangleMeshMsg(new_mesh);

  // Create msg
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = ros::Time::now();
  new_msg.header.frame_id = frame_id_;
  new_msg.mesh = mesh_msg;

  mesh_pub_.publish(new_msg);
  return true;
}

}  // namespace mesher_mapper
