/**
 * @file   voxblox_msg_interface.cpp
 * @brief  Interface wrapper around voxblox msg
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include "kimera_pgmo_ros/voxblox_msg_interface.h"

namespace kimera_pgmo {

VoxbloxMsgInterface::VoxbloxMsgInterface(const voxblox_msgs::Mesh& mesh)
    : active_mesh_block_(nullptr), mesh_(mesh) {
  size_t count = 0;
  for (const auto& block : mesh_.mesh_blocks) {
    mesh_blocks_.emplace_back(block.index[0], block.index[1], block.index[2]);
    block_lookup_[mesh_blocks_.back()] = count;
    ++count;
  }
}

const BlockIndices& VoxbloxMsgInterface::blockIndices() const { return mesh_blocks_; }

void VoxbloxMsgInterface::markBlockActive(const BlockIndex& block) const {
  const auto iter = block_lookup_.find(block);
  if (iter == block_lookup_.end()) {
    active_mesh_block_ = nullptr;
    return;
  }

  active_mesh_block_ = &mesh_.mesh_blocks.at(iter->second);
}

size_t VoxbloxMsgInterface::activeBlockSize() const {
  if (!active_mesh_block_) {
    return 0;
  }

  return active_mesh_block_->x.size();
}

pcl::PointXYZRGBA VoxbloxMsgInterface::getActiveVertex(size_t i) const {
  // (2*block_size), see mesh_vis.h for the slightly convoluted justification of the 2.
  pcl::PointXYZRGBA point;
  point.x = (static_cast<float>(active_mesh_block_->x.at(i)) * point_conv_factor_ +
             static_cast<float>(active_mesh_block_->index[0])) *
            mesh_.block_edge_length;
  point.y = (static_cast<float>(active_mesh_block_->y.at(i)) * point_conv_factor_ +
             static_cast<float>(active_mesh_block_->index[1])) *
            mesh_.block_edge_length;
  point.z = (static_cast<float>(active_mesh_block_->z.at(i)) * point_conv_factor_ +
             static_cast<float>(active_mesh_block_->index[2])) *
            mesh_.block_edge_length;
  point.r = active_mesh_block_->r[i];
  point.g = active_mesh_block_->g[i];
  point.b = active_mesh_block_->b[i];
  point.a = std::numeric_limits<uint8_t>::max();
  return point;
}

MeshInterface::Ptr VoxbloxMsgInterface::clone() const {
  return std::make_shared<VoxbloxMsgInterface>(*this);
}

}  // namespace kimera_pgmo
