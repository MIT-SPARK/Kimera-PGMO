/**
 * @file   MeshCompression.h
 * @brief  Simplify and reconstruct meshes
 * @author Yun Chang
 */

#pragma once

#include <ros/ros.h>
#include <boost/shared_ptr.hpp>
#include <map>
#include <memory>

#include <pcl/PolygonMesh.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "kimera_pgmo/utils/CommonStructs.h"

namespace kimera_pgmo {

class MeshCompression {
 public:
  typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;

  MeshCompression(double resolution) : resolution_(resolution) {}
  ~MeshCompression() = default;

  /*! \brief Get the vertices of the compressed full mesh
   *  - vertices: pointer to vertices of full compressed mesh
   */
  inline void getVertices(PointCloud::Ptr vertices) {
    *vertices = all_vertices_;
  }

  /*! \brief Get the vertices currently in the octree (actively being checked
   * for duplication according to resolution)
   *  - vertices: pointer to vertices in octree
   */
  inline void getActiveVertices(PointCloudXYZ::Ptr vertices) {
    *vertices = *active_vertices_xyz_;
  }

  /*! \brief Get the surfaces of the compressed full mesh
   *  - vertices: pointer to surfaces of full compressed mesh
   */
  inline void getStoredPolygons(
      boost::shared_ptr<std::vector<pcl::Vertices> > polygons) {
    *polygons = polygons_;
  }

  /*! \brief Get the timestamps of the active vertices (time of the msg from
   * which the vertices were inserted )
   *  - timestamps: vector of the timestamps indices corresponding to active
   * vertices
   */
  inline void getActiveVerticesTimestamps(
      boost::shared_ptr<std::vector<double> > timestamps) {
    *timestamps = vertices_latest_time_;
  }

  inline size_t getNumVertices() const { return all_vertices_.size(); }

  inline const std::vector<size_t>& getActiveVerticesIndex() const {
    return active_vertices_index_;
  }

 protected:
  // Vertices in octree (vertices of "active" part of mesh)
  PointCloudXYZ::Ptr active_vertices_xyz_;
  // All verices
  PointCloud all_vertices_;
  // Maps index of active vertices to index of all vertices
  std::vector<size_t> active_vertices_index_;
  // Mesh surfaces (all)
  std::vector<pcl::Vertices> polygons_;
  // Keep track of adjacent faces of active part of mesh
  std::map<size_t, std::vector<size_t> > adjacent_polygons_;

  std::vector<double> vertices_latest_time_;  // timestamps of active vertices

  double resolution_;
};
}  // namespace kimera_pgmo