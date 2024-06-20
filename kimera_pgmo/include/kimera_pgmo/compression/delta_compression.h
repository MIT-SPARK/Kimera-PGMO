/**
 * @file   delta_compression.h
 * @brief  Compute update delta for mesh using voxel hashing
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <voxblox/core/block_hash.h>

#include "kimera_pgmo/mesh_delta.h"
#include "kimera_pgmo/utils/common_structs.h"
#include "kimera_pgmo/utils/mesh_interface.h"

namespace kimera_pgmo {

struct VertexInfo {
  bool is_new = true;
  uint64_t timestamp_ns;
  pcl::PointXYZRGBA point;
  std::optional<uint32_t> label;
  size_t mesh_index;
  mutable int active_refs = 0;
  mutable int inactive_refs = 0;

  void addObservation() const;

  void archiveObservation() const;

  void removeObservation() const;

  bool notObserved() const;

  bool shouldArchive() const;
};

struct BlockInfo {
  voxblox::LongIndexSet vertices;
  uint64_t update_time;
  std::vector<size_t> indices;
};

class DeltaCompression {
 public:
  // TODO(nathan) unify with mesh compression
  using VoxbloxIndexMapping = voxblox::AnyIndexHashMapType<IndexMapping>::type;
  using VoxelInfoMap = voxblox::LongIndexHashMapType<VertexInfo>::type;
  using BlockInfoMap = voxblox::AnyIndexHashMapType<BlockInfo>::type;
  using Ptr = std::shared_ptr<DeltaCompression>;

  explicit DeltaCompression(double resolution);

  virtual ~DeltaCompression() = default;

  MeshDelta::Ptr update(MeshInterface& mesh,
                        uint64_t timestamp_ns,
                        VoxbloxIndexMapping* remapping = nullptr);

  void pruneStoredMesh(uint64_t earliest_time_ns);

  void clearArchivedBlocks(const voxblox::BlockIndexList& mesh);

 protected:
  void addPoint(const pcl::PointXYZRGBA& point,
                std::optional<uint32_t> semantic_label,
                uint64_t timestamp_ns,
                std::vector<size_t>& face_map,
                voxblox::LongIndexSet& curr_voxels);

  void removeBlockObservations(const voxblox::BlockIndex& block_index,
                               const voxblox::LongIndexSet& to_remove);

  void addActive(uint64_t stamp_ns, VoxbloxIndexMapping* remapping);

  void addActiveFaces(uint64_t timestamp_ns, VoxbloxIndexMapping* remapping);

  void addActiveVertices(uint64_t timestamp_ns);

  void pruneMeshBlocks(const voxblox::BlockIndexList& to_clear);

  void updateAndAddArchivedFaces();

  void archiveBlockFaces();

  void updateRemapping(MeshInterface& mesh, uint64_t timestamp_ns);

  bool canBeArchived(const Face& face, size_t archive_threshold) const;

  bool canBeArchived(const Face& face) const;

 protected:
  double resolution_;
  double index_scale_;

  MeshDelta::Ptr delta_;
  MeshDelta::Ptr archive_delta_;

  std::vector<size_t> active_remapping_;
  BlockInfoMap block_info_map_;
  BlockInfoMap archived_block_info_map_;
  VoxelInfoMap vertices_map_;

  std::vector<Face> archived_faces_;

  std::set<uint64_t> timestamp_cache_;

  size_t num_archived_vertices_;
  size_t num_archived_faces_;
};

}  // namespace kimera_pgmo
