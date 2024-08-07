/**
 * @file   VoxbloxMsgInterface.cpp
 * @brief  Interface wrapper around voxblox msg
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include "kimera_pgmo/utils/VoxbloxMeshInterface.h"

namespace kimera_pgmo {

VoxbloxMeshInterface::VoxbloxMeshInterface(const voxblox::MeshLayer::Ptr& mesh)
    : mesh_(mesh) {
  assert(nullptr != mesh);
  mesh_->getAllAllocatedMeshes(&mesh_blocks_);
}

const voxblox::BlockIndexList& VoxbloxMeshInterface::blockIndices() const {
  return mesh_blocks_;
}

void VoxbloxMeshInterface::markBlockActive(const voxblox::BlockIndex& block) {
  active_block_ = mesh_->getMeshPtrByIndex(block);
}

size_t VoxbloxMeshInterface::activeBlockSize() const {
  if (!active_block_) {
    return 0;
  }

  return active_block_->size();
}

pcl::PointXYZRGBA VoxbloxMeshInterface::getActiveVertex(size_t i) const {
  if (!active_block_) {
    throw std::runtime_error("no active block set");
  }

  const auto& pos = active_block_->vertices.at(i);
  const auto& color = active_block_->colors.at(i);

  pcl::PointXYZRGBA point;
  point.x = pos.x();
  point.y = pos.y();
  point.z = pos.z();
  point.r = color.r;
  point.g = color.g;
  point.b = color.b;
  point.a = color.a;
  return point;
}

SemanticVoxbloxMeshInterface::SemanticVoxbloxMeshInterface(
    const voxblox::MeshLayer::Ptr& mesh,
    const std::shared_ptr<SemanticLabelMesh>& semantics)
    : VoxbloxMeshInterface(mesh),
      semantics_(semantics),
      active_semantic_block_(nullptr) {
  assert(nullptr != semantics);
}

void SemanticVoxbloxMeshInterface::markBlockActive(const voxblox::BlockIndex& block) {
  VoxbloxMeshInterface::markBlockActive(block);
  auto iter = semantics_->find(block);
  if (iter == semantics_->end()) {
    throw std::runtime_error("semantic block does not exist");
  }

  active_semantic_block_ = &iter->second;
}

std::optional<uint32_t> SemanticVoxbloxMeshInterface::getActiveSemantics(
    size_t i) const {
  if (!active_semantic_block_) {
    throw std::runtime_error("no active block set");
  }

  return active_semantic_block_->at(i);
}

}  // namespace kimera_pgmo
