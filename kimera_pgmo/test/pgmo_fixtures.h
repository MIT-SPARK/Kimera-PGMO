#pragma once
/**
 * @file   pgmo_fixtures.h
 * @brief  Useful functions for making unit tests
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include <voxblox/mesh/mesh_layer.h>

#include "kimera_pgmo/utils/PclMeshInterface.h"
#include "kimera_pgmo/utils/voxblox_mesh_interface.h"

namespace kimera_pgmo::test {

class OrderedVoxbloxMeshInterface : public VoxbloxMeshInterface {
 public:
  OrderedVoxbloxMeshInterface(const voxblox::MeshLayer::Ptr &mesh,
                              const voxblox::BlockIndexList &blocks);
  virtual ~OrderedVoxbloxMeshInterface() = default;
};

struct BlockConfig {
  using FaceCoordinates = std::array<std::array<float, 3>, 3>;

  std::string name;
  std::array<int64_t, 3> index;
  std::vector<FaceCoordinates> faces;
  static uint8_t point_index;

  void addBlock(voxblox::MeshLayer &layer) const;

  static void resetIndex();
};

std::shared_ptr<MeshInterface> createMesh(const std::vector<BlockConfig> &configs);

pcl::PolygonMesh createSimpleMesh(double scale = 1.0);

PclMeshInterface createSimpleMeshInterface(double scale);

}  // namespace kimera_pgmo::test
