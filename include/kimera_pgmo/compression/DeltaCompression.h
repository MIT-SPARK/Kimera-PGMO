/**
 * @file   DeltaCompression.h
 * @brief  Compute update delta for mesh using voxel hashing
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/MeshDelta.h"
#include "kimera_pgmo/utils/CommonStructs.h"
#include "kimera_pgmo/utils/MeshInterface.h"

namespace kimera_pgmo {

struct VertexInfo {
  bool is_new = true;
  uint64_t timestamp_ns;
  pcl::PointXYZRGBA point;
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

using VoxelInfoMap = voxblox::LongIndexHashMapType<VertexInfo>::type;
using BlockInfoMap = voxblox::AnyIndexHashMapType<BlockInfo>::type;

class DeltaCompression {
 public:
  using Ptr = std::shared_ptr<DeltaCompression>;

  explicit DeltaCompression(double resolution);

  virtual ~DeltaCompression() = default;

  MeshDelta::Ptr update(const voxblox_msgs::Mesh& mesh,
                        uint64_t timestamp_ns,
                        VoxbloxIndexMapping* remapping = nullptr);

  MeshDelta::Ptr update(MeshInterface& mesh,
                        uint64_t timestamp_ns,
                        VoxbloxIndexMapping* remapping = nullptr);

  void pruneStoredMesh(uint64_t earliest_time_ns);

  void clearArchivedBlocks(const voxblox::BlockIndexList& mesh);

 protected:
  void addPoint(const pcl::PointXYZRGBA& point,
                uint64_t timestamp_ns,
                std::vector<size_t>& face_map,
                std::vector<size_t>& active_remapping,
                voxblox::LongIndexSet& curr_voxels);

  void removeBlockObservations(const voxblox::LongIndexSet& to_remove);

  void addActiveFaces(uint64_t timestamp_ns,
                      const std::vector<size_t>& active_remapping,
                      VoxbloxIndexMapping* remapping);

  void addActiveVertices(uint64_t timestamp_ns, std::vector<size_t>& active_remapping);

  void pruneMeshBlocks(const voxblox::BlockIndexList& to_clear);

  void addPartialFaces();

  void updateArchivedFaces(const std::map<size_t, size_t>& remapping);

  void archiveBlockFaces(const voxblox::BlockIndexList& to_clear,
                         const std::map<size_t, size_t>& remapping);

  void updateRemapping(MeshInterface& mesh,
                       uint64_t timestamp_ns,
                       VoxbloxIndexMapping* remapping);

  bool canBeArchived(const Face& face) const;

 protected:
  double resolution_;
  double index_scale_;

  MeshDelta::Ptr delta_;
  MeshDelta::Ptr archive_delta_;

  BlockInfoMap block_info_map_;
  VoxelInfoMap vertices_map_;

  std::vector<Face> archived_faces_;

  size_t num_archived_vertices_;
  size_t num_archived_faces_;
};

}  // namespace kimera_pgmo
