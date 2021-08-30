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
#include <unordered_map>
#include <vector>

#include <pcl/PolygonMesh.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <Eigen/Dense>

#include "kimera_pgmo/utils/CommonStructs.h"

namespace kimera_pgmo {

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloudXYZ;
typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;

class MeshCompression {
 public:
  MeshCompression(double resolution) : resolution_(resolution) {}
  virtual ~MeshCompression() = default;

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

  /*! \brief Compress and integrate with the full compressed mesh
   *  - input: input mesh in polygon mesh type
   *  - new_vertices: new vertices added after compression
   *  - new_triangles: new mesh surfaces (as triangles) added after compression
   *  - new_indices: indices of the vertices of the compressed partial mesh
   *  - stamp_in_sec: current time stamp in seconds
   */
  virtual void compressAndIntegrate(
      const pcl::PolygonMesh& input,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
      boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles,
      boost::shared_ptr<std::vector<size_t> > new_indices,
      boost::shared_ptr<std::unordered_map<size_t, size_t> > remapping,
      const double& stamp_in_sec = ros::Time::now().toSec());

  /*! \brief Compress and integrate with the full compressed mesh
   *  - input_vertices: vertices of input mesh
   *  - input_surfaces: surfaces of input mesh
   *  - new_vertices: new vertices added after compression
   *  - new_triangles: new mesh surfaces (as triangles) added after compression
   *  - new_indices: indices of the vertices of the compressed partial mesh
   *  - stamp_in_sec: current time stamp in seconds
   */
  virtual void compressAndIntegrate(
      const pcl::PointCloud<pcl::PointXYZRGBA>& input_vertices,
      const std::vector<pcl::Vertices>& input_surfaces,
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
      boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles,
      boost::shared_ptr<std::vector<size_t> > new_indices,
      boost::shared_ptr<std::unordered_map<size_t, size_t> > remapping,
      const double& stamp_in_sec = ros::Time::now().toSec());

  /*! \brief Discard parts of the stored compressed full mesh by detection time
   *  - earliest_time_sec: discard all vertices added earlier than this time in
   * seconds
   */
  void pruneStoredMesh(const double& earliest_time_sec);

  /*! \brief Reinitialize the compression structure (ie. octree or hash cells)
   *  - active_vertices: xyz of the active vertices
   */
  virtual void reInitializeStructure(PointCloudXYZ::Ptr active_vertices) = 0;

  /*! \brief Check if vertex exists in structure
   */
  virtual bool checkIfVertexUnique(const pcl::PointXYZ& v,
                                   int* matched_ind) const = 0;

  /*! \brief Updatae structure
   */
  virtual void updateStructure(PointCloudXYZ::Ptr vertices) = 0;

  /*! \brief Check if vertex exists in temporary structure
   */
  virtual bool checkIfVertexTempUnique(const pcl::PointXYZ& v,
                                       int* matched_ind) const = 0;

  /*! \brief Initialize temporary structure
   */
  virtual void initializeTempStructure(PointCloudXYZ::Ptr vertices) = 0;

  /*! \brief Update temporary structure
   */
  virtual void updateTempStructure(PointCloudXYZ::Ptr vertices) = 0;

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

typedef boost::shared_ptr<MeshCompression> MeshCompressionPtr;
}  // namespace kimera_pgmo