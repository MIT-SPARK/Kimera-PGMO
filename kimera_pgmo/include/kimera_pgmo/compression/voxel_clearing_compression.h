/**
 * @file   voxel_clearing_compression.h
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once

#include "kimera_pgmo/compression/mesh_compression.h"

namespace kimera_pgmo {

class VoxelClearingCompression : public MeshCompression {
 public:
  using VoxelToMeshIndex = voxblox::LongIndexHashMapType<size_t>::type;
  using BlockMap = voxblox::AnyIndexHashMapType<voxblox::LongIndexSet>::type;
  using BlockTimeMap = voxblox::AnyIndexHashMapType<double>::type;
  using BlockSizeMap = voxblox::AnyIndexHashMapType<size_t>::type;
  using BlockFaceMap = voxblox::AnyIndexHashMapType<std::vector<size_t>>::type;

  explicit VoxelClearingCompression(double resolution);

  virtual ~VoxelClearingCompression() = default;

  /*! \brief Compress and integrate with the full compressed mesh
   *  - mesh: input mesh as abstract mesh type
   *  - new_vertices: new vertices added after compression
   *  - new_triangles: new mesh surfaces (as triangles) added after compression
   *  - new_indices: indices of the vertices of the compressed partial mesh
   *  - stamp_in_sec: current time stamp in seconds
   */
  virtual void compressAndIntegrate(const MeshInterface& mesh,
                                    PointCloud& new_vertices,
                                    std::vector<pcl::Vertices>& new_triangles,
                                    std::vector<size_t>& new_indices,
                                    VoxbloxIndexMapping& remapping,
                                    double stamp_in_sec) override;

  void pruneStoredMesh(double earliest_time_sec) override;

  // TODO(nathan) push interface to different class
  inline void reInitializeStructure(PointCloudXYZ::Ptr /*active_vertices*/) override {
    throw std::logic_error("not implemented");
  }

  inline bool checkIfVertexUnique(const pcl::PointXYZ& /*v*/,
                                  int* /*matched_ind*/) const override {
    throw std::logic_error("not implemented");
  }

  inline void updateStructure(PointCloudXYZ::Ptr /*vertices*/) override {
    throw std::logic_error("not implemented");
  }

  bool checkIfVertexTempUnique(const pcl::PointXYZ& /*v*/,
                               int* /*matched_ind*/) const override {
    throw std::logic_error("not implemented");
  }

  void initializeTempStructure(PointCloudXYZ::Ptr /*vertices*/) override {
    throw std::logic_error("not implemented");
  }

  void updateTempStructure(PointCloudXYZ::Ptr /*vertices*/) override {
    throw std::logic_error("not implemented");
  }

  inline std::vector<size_t> getInvalidIndices() const override { return empty_slots_; }

  void clearArchivedBlocks(const BlockIndices& indices) override;

 protected:
  void pruneMeshBlocks(const BlockIndices& to_clear);

  void updateRemapping(const MeshInterface& mesh,
                       double stamp_in_sec,
                       VoxbloxIndexMapping& remapping);

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

using VoxelClearingCompressionPtr = std::shared_ptr<VoxelClearingCompression>;

}  // namespace kimera_pgmo
