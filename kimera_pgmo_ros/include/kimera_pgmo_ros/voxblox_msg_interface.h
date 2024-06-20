/**
 * @file   voxblox_msg_interface.cpp
 * @brief  Interface wrapper around voxblox msg
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <voxblox/core/block_hash.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/utils/mesh_interface.h"

namespace kimera_pgmo {

class VoxbloxMsgInterface : public MeshInterface {
 public:
  VoxbloxMsgInterface(const voxblox_msgs::Mesh* const mesh);

  const voxblox::BlockIndexList& blockIndices() const override;

  void markBlockActive(const voxblox::BlockIndex& block) const override;

  size_t activeBlockSize() const override;

  pcl::PointXYZRGBA getActiveVertex(size_t i) const override;

  MeshInterface::Ptr clone() const override {
    return std::make_shared<VoxbloxMsgInterface>(*this);
  }

 private:
  voxblox::AnyIndexHashMapType<size_t>::type block_lookup_;
  mutable const voxblox_msgs::MeshBlock* active_mesh_block_;
  voxblox::BlockIndexList mesh_blocks_;
  const voxblox_msgs::Mesh* const mesh_;
};

}  // namespace kimera_pgmo
