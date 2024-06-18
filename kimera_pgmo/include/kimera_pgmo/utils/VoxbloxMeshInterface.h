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

  virtual void markBlockActive(const voxblox::BlockIndex& block) override;

  size_t activeBlockSize() const override;

  pcl::PointXYZRGBA getActiveVertex(size_t i) const override;

 protected:
  voxblox::BlockIndexList mesh_blocks_;
  voxblox::MeshLayer::Ptr mesh_;
  voxblox::Mesh::Ptr active_block_;
};

class SemanticVoxbloxMeshInterface : public VoxbloxMeshInterface {
 public:
  using SemanticLabelMesh = voxblox::AnyIndexHashMapType<std::vector<uint32_t>>::type;

  SemanticVoxbloxMeshInterface(const voxblox::MeshLayer::Ptr& mesh,
                               const std::shared_ptr<SemanticLabelMesh>& semantics);

  void markBlockActive(const voxblox::BlockIndex& block) override;

  bool hasSemantics() const override { return true; }

  std::optional<uint32_t> getActiveSemantics(size_t i) const override;

 private:
  std::shared_ptr<SemanticLabelMesh> semantics_;
  const std::vector<uint32_t>* active_semantic_block_;
};

}  // namespace kimera_pgmo
