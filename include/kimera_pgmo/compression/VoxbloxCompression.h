/**
 * @file   VoxbloxCompression.h
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 */
#pragma once

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh.h>

#include "kimera_pgmo/compression/MeshCompression.h"

namespace kimera_pgmo {

class VoxbloxCompression : public MeshCompression {
 public:
  VoxbloxCompression(double resolution);
  virtual~VoxbloxCompression();

  /*! \brief Reinitialize the cell hash map
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
  // Grid hash from voxblox
  voxblox::LongIndexHashMapType<size_t>::type cell_hash_;
  voxblox::LongIndexHashMapType<size_t>::type temp_cell_hash_;
};

typedef std::shared_ptr<VoxbloxCompression> VoxbloxCompressionPtr;
}  // namespace kimera_pgmo
