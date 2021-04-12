/**
 * @file   OctreeCompression.h
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#pragma once

#include <boost/shared_ptr.hpp>
#include <string>
#include <vector>

#include <ros/ros.h>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/octree/octree_search.h>
#include <pcl_ros/point_cloud.h>

namespace kimera_pgmo {

class OctreeCompression {
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
      boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles,
      boost::shared_ptr<std::vector<size_t> > new_indices,
      const double& stamp_in_sec = ros::Time::now().toSec());

  /*! \brief Compress and integrate with the full compressed mesh
   *  - input_vertices: vertices of input mesh
   *  - input_surfaces: surfaces of input mesh
   *  - new_vertices: new vertices added after compression
   *  - new_triangles: new mesh surfaces (as triangles) added after compression
   *  - new_indices: indices of the vertices of the compressed partial mesh
   *  - stamp_in_sec: current time stamp in seconds
   */
  void compressAndIntegrate(
      const pcl::PointCloud<pcl::PointXYZRGBA>& input_vertices,
      const std::vector<pcl::Vertices>& input_surfaces,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
      boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles,
      boost::shared_ptr<std::vector<size_t> > new_indices,
      const double& stamp_in_sec = ros::Time::now().toSec());

  /*! \brief Discard parts of the stored compressed full mesh by detection time
   *  - earliest_time_sec: discard all vertices added earlier than this time in
   * seconds
   */
  void pruneStoredMesh(const double& earliest_time_sec);

  /*! \brief Get the vertices of the compressed full mesh
   *  - vertices: pointer to vertices of full compressed mesh
   */
  inline void getVertices(PointCloud::Ptr vertices) {
    *vertices = *all_vertices_;
  }

  /*! \brief Get the vertices currently in the octree (actively being checked
   * for duplication according to resolution)
   *  - vertices: pointer to vertices in octree
   */
  inline void getActiveVertices(PointCloud::Ptr vertices) {
    *vertices = *active_vertices_;
  }

  /*! \brief Get the surfaces of the compressed full mesh
   *  - vertices: pointer to surfaces of full compressed mesh
   */
  inline void getStoredPolygons(std::vector<pcl::Vertices>* polygons) {
    *polygons = polygons_;
  }

  /*! \brief Get the timestamps of the active vertices (time of the msg from
   * which the vertices were inserted )
   *  - timestamps: vector of the timestamps indices corresponding to active
   * vertices
   */
  inline void getActiveVerticesTimestamps(std::vector<double>* timestamps) {
    *timestamps = vertices_latest_time_;
  }

  inline size_t getNumVertices() const { return all_vertices_->size(); }

 protected:
  PointCloud::Ptr active_vertices_;  // vertices in octree
  PointCloud::Ptr all_vertices_;     // all verices
  std::vector<size_t> active_vertices_index_;
  // Index of active vertices in all vertices
  std::vector<pcl::Vertices> polygons_;
  // Keep track of adjacent polygons of vertices
  std::vector<std::vector<pcl::Vertices> > adjacent_polygons_;
  // Octree of compressor
  Octree::Ptr octree_;

  std::vector<double> vertices_latest_time_;  // timestamps of active vertices

  double octree_resolution_;
};

typedef boost::shared_ptr<OctreeCompression> OctreeCompressionPtr;
}  // namespace kimera_pgmo