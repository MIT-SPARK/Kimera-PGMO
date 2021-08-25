/**
 * @file   OctreeCompression.h
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#pragma once

#include <string>
#include <unordered_map>
#include <vector>

#include "kimera_pgmo/compression/MeshCompression.h"

namespace kimera_pgmo {

class OctreeCompression : public MeshCompression {
 public:
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
  void compressAndIntegrate(
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

 protected:
  // Octree of compressor
  Octree::Ptr octree_;
};

typedef boost::shared_ptr<OctreeCompression> OctreeCompressionPtr;
}  // namespace kimera_pgmo
