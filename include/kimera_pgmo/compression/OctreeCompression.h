/**
 * @file   OctreeCompression.h
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#pragma once

#include "kimera_pgmo/compression/MeshCompression.h"

namespace kimera_pgmo {

class OctreeCompression : public MeshCompression {
 public:
  OctreeCompression(double resolution);
  virtual~OctreeCompression();

  /*! \brief Reinitialize the octree
   *  - active_vertices: xyz of the active vertices
   */
  void reInitializeStructure(PointCloudXYZ::Ptr active_vertices) override;

  /*! \brief Check if vertex exists in structure
   */
  bool checkIfVertexUnique(const pcl::PointXYZ& v,
                           int* matched_ind) const override;

  /*! \brief Updatae structure
   */
  void updateStructure(PointCloudXYZ::Ptr vertices) override;

  /*! \brief Check if vertex exists in temporary structure
   */
  bool checkIfVertexTempUnique(const pcl::PointXYZ& v,
                               int* matched_ind) const override;

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
