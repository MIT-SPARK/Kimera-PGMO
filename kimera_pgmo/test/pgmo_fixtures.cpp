/**
 * @file   pgmo_fixtures.cpp
 * @brief  Useful functions for making unit tests
 * @author Yun Chang
 * @author Nathan Hughes
 */

#include "pgmo_fixtures.h"

namespace kimera_pgmo::test {

uint8_t BlockConfig::point_index = 0;

MeshBlock::MeshBlock(const float block_size, const BlockIndex &index)
    : Block(block_size, index) {}

OrderedBlockMeshInterface::OrderedBlockMeshInterface(
    const std::shared_ptr<MeshLayer> &mesh,
    const BlockIndices &blocks) {
  mesh_blocks_ = blocks;
  mesh_ = mesh;
}

const BlockIndices &OrderedBlockMeshInterface::blockIndices() const {
  return mesh_blocks_;
}

void OrderedBlockMeshInterface::markBlockActive(const BlockIndex &block) const {
  active_block_ = mesh_->getBlockPtr(block);
}

size_t OrderedBlockMeshInterface::activeBlockSize() const {
  if (!active_block_) {
    return 0;
  }

  return active_block_->vertices.size();
};

pcl::PointXYZRGBA OrderedBlockMeshInterface::getActiveVertex(size_t i) const {
  if (!active_block_) {
    throw std::runtime_error("no active block set");
  }
  return active_block_->vertices[i];
}

std::shared_ptr<MeshInterface> OrderedBlockMeshInterface::clone() const {
  return std::make_shared<OrderedBlockMeshInterface>(
      std::make_shared<MeshLayer>(*mesh_), mesh_blocks_);
}

inline void addPointToBlock(MeshBlock &block,
                            const std::array<float, 3> &point,
                            uint8_t point_index) {
  pcl::PointXYZRGBA pcl_point;
  pcl_point.x = point[0];
  pcl_point.y = point[1];
  pcl_point.z = point[2];
  pcl_point.r = point_index;
  pcl_point.g = point_index;
  pcl_point.b = point_index;
  pcl_point.a = 255;
  block.vertices.push_back(pcl_point);
}

void BlockConfig::addBlock(MeshLayer &layer) const {
  BlockIndex block_idx(index[0], index[1], index[2]);
  auto &mesh = layer.allocateBlock(block_idx);

  for (const auto &face : faces) {
    addPointToBlock(mesh, face[0], point_index);
    ++point_index;
    addPointToBlock(mesh, face[1], point_index);
    ++point_index;
    addPointToBlock(mesh, face[2], point_index);
    ++point_index;
  }
}

void BlockConfig::resetIndex() { point_index = 0; }

std::shared_ptr<MeshInterface> createMesh(const std::vector<BlockConfig> &configs) {
  auto mesh = std::make_shared<MeshLayer>(1.0);
  spatial_hash::BlockIndices order;
  for (const auto &config : configs) {
    config.addBlock(*mesh);

    const BlockIndex block_idx(config.index[0], config.index[1], config.index[2]);
    order.push_back(block_idx);
  }

  return std::make_shared<OrderedBlockMeshInterface>(mesh, order);
}

pcl::PolygonMesh createSimpleMesh(double scale) {
  // Create simple pcl mesh
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZRGBA> ptcld;
  pcl::PointXYZRGBA v0, v1, v2, v3, v4;
  v0.x = scale * 0;
  v0.y = scale * 0;
  v0.z = scale * 0;
  v0.r = 23;
  v0.g = 24;
  v0.b = 122;
  v0.a = 255;

  v1.x = scale * 1;
  v1.y = scale * 0;
  v1.z = scale * 0;
  v1.r = 33;
  v1.g = 34;
  v1.b = 52;
  v1.a = 255;

  v2.x = scale * 0;
  v2.y = scale * 1;
  v2.z = scale * 0;
  v2.r = 12;
  v2.g = 144;
  v2.b = 22;
  v2.a = 255;

  v3.x = scale * 1;
  v3.y = scale * 1;
  v3.z = scale * 0;
  v3.r = 0;
  v3.g = 14;
  v3.b = 0;
  v3.a = 255;

  v4.x = scale * 0;
  v4.y = scale * 0;
  v4.z = scale * 1;
  v4.r = 144;
  v4.g = 0;
  v4.b = 12;
  v4.a = 255;

  ptcld.points.push_back(v0);
  ptcld.points.push_back(v1);
  ptcld.points.push_back(v2);
  ptcld.points.push_back(v3);
  ptcld.points.push_back(v4);
  pcl::toPCLPointCloud2(ptcld, mesh.cloud);

  pcl::Vertices tri_1, tri_2, tri_3, tri_4;
  tri_1.vertices = std::vector<uint>{0, 1, 2};
  tri_2.vertices = std::vector<uint>{1, 3, 2};
  tri_3.vertices = std::vector<uint>{0, 1, 4};
  tri_4.vertices = std::vector<uint>{0, 4, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1, tri_2, tri_3, tri_4};

  return mesh;
}

PclMeshInterface createSimpleMeshInterface(double scale) {
  return PclMeshInterface(createSimpleMesh(scale));
}

}  // namespace kimera_pgmo::test
