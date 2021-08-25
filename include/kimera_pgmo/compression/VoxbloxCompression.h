/**
 * @file   VoxbloxCompression.h
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 */
#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh.h>

#include "kimera_pgmo/compression/MeshCompression.h"

namespace kimera_pgmo {

class VoxbloxCompression : public MeshCompression {
 public:
  VoxbloxCompression(double resolution);
  ~VoxbloxCompression();

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
  // Grid hash from voxblox
  voxblox::LongIndexHashMapType<size_t>::type cell_hash_;
};

typedef boost::shared_ptr<VoxbloxCompression> VoxbloxCompressionPtr;
}  // namespace kimera_pgmo
