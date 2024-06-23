/**
 * @file   block_compression.h
 * @brief  Combine and simplify meshes based on voxel/mesh block hashing
 * @author Yun Chang
 */
#pragma once

#include <spatial_hash/grid.h>

#include "kimera_pgmo/compression/mesh_compression.h"
#include "kimera_pgmo/hashing.h"

namespace kimera_pgmo {

class BlockCompression : public MeshCompression {
 public:
  using CellHash = spatial_hash::LongIndexHashMap<size_t>;
  using Ptr = std::shared_ptr<BlockCompression>;

  BlockCompression(double resolution);

  virtual ~BlockCompression() = default;

  /*! \brief Reinitialize the cell hash map
   *  - active_vertices: xyz of the active vertices
   */
  void reInitializeStructure(PointCloudXYZ::Ptr active_vertices) override;

  /*! \brief Check if vertex exists in structure
   */
  bool checkIfVertexUnique(const pcl::PointXYZ& v, int* matched_ind) const override;

  /*! \brief Updatae structure
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
  // Block grid hash.
  spatial_hash::Grid<LongIndex> grid_;
  CellHash cell_hash_;
  CellHash temp_cell_hash_;

  LongIndex toIndex(const pcl::PointXYZ& p) const;
};

}  // namespace kimera_pgmo
