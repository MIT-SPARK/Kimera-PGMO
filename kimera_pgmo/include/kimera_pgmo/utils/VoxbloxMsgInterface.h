/**
 * @file   VoxbloxMsgInterface.cpp
 * @brief  Interface wrapper around voxblox msg
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <voxblox/core/block_hash.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/utils/MeshInterface.h"

namespace kimera_pgmo {

class VoxbloxMsgInterface : public MeshInterface {
 public:
  VoxbloxMsgInterface(const voxblox_msgs::Mesh* const mesh);

  const voxblox::BlockIndexList& blockIndices() const override;

  void markBlockActive(const voxblox::BlockIndex& block) override;

  size_t activeBlockSize() const override;

  pcl::PointXYZRGBA getActiveVertex(size_t i) const override;

 private:
  voxblox::AnyIndexHashMapType<size_t>::type block_lookup_;
  const voxblox_msgs::MeshBlock* active_mesh_block_;
  voxblox::BlockIndexList mesh_blocks_;
  const voxblox_msgs::Mesh* const mesh_;
};

}  // namespace kimera_pgmo
