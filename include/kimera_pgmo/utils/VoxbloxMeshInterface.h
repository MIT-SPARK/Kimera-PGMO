/**
 * @file   VoxbloxMeshInterface.h
 * @brief  Interface wrapper around voxblox mesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <voxblox/mesh/mesh_layer.h>

#include "kimera_pgmo/utils/MeshInterface.h"

namespace kimera_pgmo {

class VoxbloxMeshInterface : public MeshInterface {
 public:
  VoxbloxMeshInterface(const voxblox::MeshLayer::Ptr& mesh);

  const voxblox::BlockIndexList& blockIndices() const override;

  void markBlockActive(const voxblox::BlockIndex& block) override;

  size_t activeBlockSize() const override;

  pcl::PointXYZRGBA getActiveVertex(size_t i) const override;

 private:
  voxblox::BlockIndexList mesh_blocks_;
  voxblox::MeshLayer::Ptr mesh_;
  voxblox::Mesh::Ptr active_block_;
};

}  // namespace kimera_pgmo
