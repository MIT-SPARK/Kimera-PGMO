/**
 * @file   octree_compression.h
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#pragma once

#include <pcl/octree/octree_search.h>

#include "kimera_pgmo/compression/mesh_compression.h"

namespace kimera_pgmo {

typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;

/*! \brief Check if a point is within the bounding box of an octree structure
 *  - octree: pcl octree type
 *  - p: point to query
 */
template <class point_type>
bool InOctreeBoundingBox(const pcl::octree::OctreePointCloudSearch<point_type>& octree,
                         const point_type& p) {
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
  return (p.x >= min_x && p.x <= max_x) && (p.y >= min_y && p.y <= max_y) &&
         (p.z >= min_z && p.z <= max_z);
}

class OctreeCompression : public MeshCompression {
 public:
  OctreeCompression(double resolution);
  virtual ~OctreeCompression();

  /*! \brief Reinitialize the octree
   *  - active_vertices: xyz of the active vertices
   */
  void reInitializeStructure(PointCloudXYZ::Ptr active_vertices) override;

  /*! \brief Check if vertex exists in structure
   */
  bool checkIfVertexUnique(const pcl::PointXYZ& v, int* matched_ind) const override;

  /*! \brief Update structure
   */
  void updateStructure(PointCloudXYZ::Ptr vertices) override;

  /*! \brief Check if vertex exists in temporary structure
   */
  bool checkIfVertexTempUnique(const pcl::PointXYZ& v, int* matched_ind) const override;

  /*! \brief Initialize temporary structure
   */
  void initializeTempStructure(PointCloudXYZ::Ptr vertices) override;

  /*! \brief Update temporary structure
   */
  void updateTempStructure(PointCloudXYZ::Ptr vertices) override;

 protected:
  // Octree of compressor
  Octree::Ptr octree_;
  Octree::Ptr temp_octree_;
};

typedef std::shared_ptr<OctreeCompression> OctreeCompressionPtr;
}  // namespace kimera_pgmo
