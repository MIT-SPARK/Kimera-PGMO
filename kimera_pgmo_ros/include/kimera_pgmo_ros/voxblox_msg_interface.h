/**
 * @file   voxblox_msg_interface.h
 * @brief  Interface wrapper around voxblox msg
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/utils/mesh_interface.h"

namespace kimera_pgmo {

class VoxbloxMsgInterface : public MeshInterface {
 public:
  VoxbloxMsgInterface(const voxblox_msgs::Mesh& mesh);

  const BlockIndices& blockIndices() const override;

  void markBlockActive(const BlockIndex& block) const override;

  size_t activeBlockSize() const override;

  pcl::PointXYZRGBA getActiveVertex(size_t i) const override;

  MeshInterface::Ptr clone() const override;

 private:
  BlockIndexMap<size_t> block_lookup_;
  mutable const voxblox_msgs::MeshBlock* active_mesh_block_;
  BlockIndices mesh_blocks_;
  const voxblox_msgs::Mesh& mesh_;
  inline static constexpr float point_conv_factor_ =
      2.0f / std::numeric_limits<uint16_t>::max();
};

}  // namespace kimera_pgmo