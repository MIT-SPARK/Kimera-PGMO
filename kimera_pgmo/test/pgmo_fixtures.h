#pragma once
/**
 * @file   pgmo_fixtures.h
 * @brief  Useful functions for making unit tests
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include <spatial_hash/block_layer.h>

#include "kimera_pgmo/hashing.h"
#include "kimera_pgmo/utils/pcl_mesh_interface.h"

namespace kimera_pgmo::test {

struct MeshBlock : spatial_hash::Block {
  MeshBlock(const float block_size, const BlockIndex &index);
  std::vector<pcl::PointXYZRGBA> vertices;
};

using MeshLayer = spatial_hash::BlockLayer<MeshBlock>;

/**
 * @brief Interface for testing meshes.
 * TODO(lschmid): This simply hallucinates a block-grid-mesh. Clean up in the future
 * potentially for more general utests.
 */
class OrderedBlockMeshInterface : public MeshInterface {
 public:

  OrderedBlockMeshInterface(const std::shared_ptr<MeshLayer> &mesh,
                            const BlockIndices &blocks);
  virtual ~OrderedBlockMeshInterface() = default;

  const BlockIndices &blockIndices() const override;

  void markBlockActive(const BlockIndex &block) const override;

  size_t activeBlockSize() const override;

  pcl::PointXYZRGBA getActiveVertex(size_t i) const override;
    
    std::shared_ptr<MeshInterface> clone() const override;

 protected:
  BlockIndices mesh_blocks_;
  std::shared_ptr<MeshLayer> mesh_;
  mutable std::shared_ptr<MeshBlock> active_block_;
};

struct BlockConfig {
  using FaceCoordinates = std::array<std::array<float, 3>, 3>;

  std::string name;
  std::array<int64_t, 3> index;
  std::vector<FaceCoordinates> faces;
  static uint8_t point_index;

  void addBlock(MeshLayer &layer) const;

  static void resetIndex();
};

std::shared_ptr<MeshInterface> createMesh(const std::vector<BlockConfig> &configs);

pcl::PolygonMesh createSimpleMesh(double scale = 1.0);

PclMeshInterface createSimpleMeshInterface(double scale);

}  // namespace kimera_pgmo::test
