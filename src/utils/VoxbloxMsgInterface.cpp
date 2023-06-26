/**
 * @file   VoxbloxMsgInterface.cpp
 * @brief  Interface wrapper around voxblox msg
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include "kimera_pgmo/utils/VoxbloxMsgInterface.h"

#include "kimera_pgmo/utils/VoxbloxUtils.h"

namespace kimera_pgmo {

VoxbloxMsgInterface::VoxbloxMsgInterface(const voxblox_msgs::Mesh* const mesh)
    : active_mesh_block_(nullptr), mesh_(mesh) {
  assert(nullptr != mesh);

  size_t count = 0;
  for (const auto& block : mesh_->mesh_blocks) {
    mesh_blocks_.push_back(
        voxblox::BlockIndex(block.index[0], block.index[1], block.index[2]));
    block_lookup_[mesh_blocks_.back()] = count;
    ++count;
  }
}

const voxblox::BlockIndexList& VoxbloxMsgInterface::blockIndices() const {
  return mesh_blocks_;
}

void VoxbloxMsgInterface::markBlockActive(const voxblox::BlockIndex& block) {
  const auto iter = block_lookup_.find(block);
  if (iter == block_lookup_.end()) {
    active_mesh_block_ = nullptr;
    return;
  }

  active_mesh_block_ = &mesh_->mesh_blocks.at(iter->second);
}

size_t VoxbloxMsgInterface::activeBlockSize() const {
  if (!active_mesh_block_) {
    return 0;
  }

  return active_mesh_block_->x.size();
}

pcl::PointXYZRGBA VoxbloxMsgInterface::getActiveVertex(size_t i) const {
  return ExtractPoint(*active_mesh_block_, mesh_->block_edge_length, i);
}

}  // namespace kimera_pgmo
