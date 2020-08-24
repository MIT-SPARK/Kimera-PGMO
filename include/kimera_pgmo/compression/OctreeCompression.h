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

namespace kimera_pgmo {

class OctreeCompression {
  friend class OctreeCompressionTest;

 public:
  typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZRGBA> Octree;

  OctreeCompression(double resolution);
  ~OctreeCompression();

  /*! \brief Compress and integrate with the full compressed mesh
   *  - input: input mesh in polygon mesh type
   *  - new_vertices: new vertices added after compression
   *  - new_triangles: new mesh surfaces (as triangles) added after compression
   *  - new_indices: indices of the vertices of the compressed partial mesh
   *  - stamp_in_sec: current time stamp in seconds
   */
  void compressAndIntegrate(
      const pcl::PolygonMesh& input,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
      std::vector<pcl::Vertices>* new_triangles,
      std::vector<size_t>* new_indices,
      const double& stamp_in_sec = ros::Time::now().toSec());

  /*! \brief Discard parts of the stored compressed full mesh by detection time
   *  - earliest_time_sec: discard all vertices added earlier than this time in
   * seconds
   */
  void pruneStoredMesh(const double& earliest_time_sec);

  inline void getVertices(PointCloud::Ptr vertices) {
    *vertices = *all_vertices_;
  }

  inline void getActiveVertices(PointCloud::Ptr vertices) {
    *vertices = *active_vertices_;
  }

  inline void getStoredPolygons(std::vector<pcl::Vertices>* polygons) {
    *polygons = polygons_;
  }

  inline void getVertexTimestamps(std::vector<double>* timestamps) {
    *timestamps = vertices_latest_time_;
  }

 protected:
  PointCloud::Ptr active_vertices_;  // vertices in octree
  PointCloud::Ptr all_vertices_;     // all verices
  std::vector<size_t> active_vertices_index_;
  // index of active vertices in all vertices
  std::vector<pcl::Vertices> polygons_;
  Octree::Ptr octree_;

  std::vector<double> vertices_latest_time_;  // timestamps of active vertices

  double octree_resolution_;
};
}  // namespace kimera_pgmo