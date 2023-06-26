/**
 * @file   VoxelClearingCompression.h
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>
#include <voxblox/mesh/mesh.h>

#include "kimera_pgmo/compression/MeshCompression.h"

namespace kimera_pgmo {

using IndexSet = voxblox::IndexSet;
using VoxelToMeshIndex = voxblox::LongIndexHashMapType<size_t>::type;
using BlockMap = voxblox::AnyIndexHashMapType<voxblox::LongIndexSet>::type;
using BlockTimeMap = voxblox::AnyIndexHashMapType<double>::type;
using BlockSizeMap = voxblox::AnyIndexHashMapType<size_t>::type;
using BlockFaceMap = voxblox::AnyIndexHashMapType<std::vector<size_t>>::type;
using voxblox::BlockIndexList;

class VoxelClearingCompression : public MeshCompression {
 public:
  explicit VoxelClearingCompression(double resolution);

  virtual ~VoxelClearingCompression() = default;

  void compressAndIntegrate(const voxblox_msgs::Mesh &mesh,
                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
                            std::shared_ptr<std::vector<pcl::Vertices>> new_triangles,
                            std::shared_ptr<std::vector<size_t>> new_indices,
                            std::shared_ptr<VoxbloxIndexMapping> remapping,
                            const double &stamp_in_sec) override;

  void pruneStoredMesh(const double &earliest_time_sec) override;

  inline void reInitializeStructure(PointCloudXYZ::Ptr /*active_vertices*/) override {
    throw std::logic_error("not implemented");
  }

  inline bool checkIfVertexUnique(const pcl::PointXYZ & /*v*/,
                                  int * /*matched_ind*/) const override {
    throw std::logic_error("not implemented");
  }

  inline void updateStructure(PointCloudXYZ::Ptr /*vertices*/) override {
    throw std::logic_error("not implemented");
  }

  bool checkIfVertexTempUnique(const pcl::PointXYZ & /*v*/,
                               int * /*matched_ind*/) const override {
    throw std::logic_error("not implemented");
  }

  void initializeTempStructure(PointCloudXYZ::Ptr /*vertices*/) override {
    throw std::logic_error("not implemented");
  }

  void updateTempStructure(PointCloudXYZ::Ptr /*vertices*/) override {
    throw std::logic_error("not implemented");
  }

  inline std::vector<size_t> getInvalidIndices() const override { return empty_slots_; }

  void clearArchivedBlocks(const voxblox_msgs::Mesh &mesh) override;

 protected:
  void pruneMeshBlocks(const BlockIndexList &to_clear);

  void updateRemapping(const voxblox_msgs::Mesh &mesh,
                       double stamp_in_sec,
                       std::shared_ptr<VoxbloxIndexMapping> remapping);

  void updateVertices();

  void updateActiveIndices();

  std::vector<pcl::Vertices> archived_polygons_;

  BlockMap prev_meshes_;
  BlockFaceMap block_face_map_;
  BlockTimeMap block_update_times_;

  VoxelToMeshIndex vertices_map_;
  std::map<size_t, size_t> indices_to_active_refs_;
  std::map<size_t, size_t> indices_to_inactive_refs_;

  std::vector<size_t> empty_slots_;
  size_t max_index_ = 0;
  size_t archived_polygon_size_ = 0;
};

typedef std::shared_ptr<VoxelClearingCompression> VoxelClearingCompressionPtr;

}  // namespace kimera_pgmo
