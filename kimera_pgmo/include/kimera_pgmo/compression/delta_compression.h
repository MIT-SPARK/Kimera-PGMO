/**
 * @file   delta_compression.h
 * @brief  Compute update delta for mesh using voxel hashing
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include "kimera_pgmo/hashing.h"
#include "kimera_pgmo/mesh_delta.h"
#include "kimera_pgmo/utils/common_structs.h"
#include "kimera_pgmo/utils/mesh_interface.h"

namespace kimera_pgmo {

struct RedunancyChecker;

//! @brief Tracking info for every vertex
struct VertexInfo {
  //! @brief Last timestamp vertex was updated
  uint64_t timestamp_ns;
  //! @brief Current vertex position and color
  pcl::PointXYZRGBA point;
  //! @brief Current vertex semantic label
  std::optional<uint32_t> label;
  //! @brief Current vertex index in integrated mesh
  size_t mesh_index;
  //! @brief Was the vertex newly observed this pass
  bool is_new = true;
  //! @brief Last integration pass the vertex was updated
  uint16_t sequence_number = 0;
  //! @brief Number of active blocks pointing at the vertex
  mutable int active_refs = 0;
  //! @brief Whether or not an archived block is pointing at the vertex
  mutable bool needs_archive = false;

  //! @brief Increment the ref count
  void addObservation() const;
  //! @brief Decrement the ref count
  void removeObservation() const;
  //! @brief Decrement the ref count and mark the vertex for archival
  void archiveObservation() const;
  //! @brief Return if the vertex is no longer observed by any block
  bool notObserved() const;
  //! @brief Return whether the vertex is marked for archival and has no active refs
  bool shouldArchive() const;
};

//! @brief Tracking struct for every block in the spatial grid used by the compression
struct BlockInfo {
  //! @brief All vertices belonging to the block
  spatial_hash::LongIndexSet vertices;
  //! @brief Last time the block was updated
  uint64_t update_time_ns;
  //! @brief Current flat list of indices
  std::vector<size_t> indices;
  //! @brief Last integration pass the block was updated
  uint16_t sequence_number = 0;
};

class DeltaCompression {
 public:
  using VoxelInfoMap = LongIndexMap<VertexInfo>;
  using BlockInfoMap = BlockIndexMap<BlockInfo>;
  using Ptr = std::shared_ptr<DeltaCompression>;
  using BlockFilter =
      std::function<bool(const spatial_hash::BlockIndex&, const BlockInfo&)>;

  /**
   * @brief Construct a mesh compressor at the provided spatial resolution
   */
  explicit DeltaCompression(double resolution);

  virtual ~DeltaCompression() = default;

  /**
   * @brief Integrate a new mesh update into the compressor
   * @param mesh Newest mesh to integrate
   * @param timestamp_ns Timestamp the mesh was generated at
   * @param remapping Optional output remapping between input mesh and integrated mesh
   */
  MeshDelta::Ptr update(MeshInterface& mesh,
                        uint64_t timestamp_ns,
                        HashedIndexMapping* remapping = nullptr);

  /**
   * @brief Archive blocks that are older than a cutoff time
   * @param earliest_time_ns Blocks updated before this timestamp will be archived
   */
  void archiveBlocksByTime(uint64_t earliest_time_ns);

  /**
   * @brief Archive blocks in the underlying spatial grid
   * @param should_archive Filter function that returns true if a block should be
   * archived
   */
  void archiveBlocks(const BlockFilter& should_archive);

 protected:
  void addPoint(const pcl::PointXYZRGBA& point,
                std::optional<uint32_t> semantic_label,
                uint64_t timestamp_ns,
                std::vector<size_t>& face_map,
                spatial_hash::LongIndexSet& curr_voxels);

  void removeBlockObservations(const spatial_hash::LongIndexSet& to_remove);

  void addActive(uint64_t stamp_ns, HashedIndexMapping* remapping);

  void addActiveFaces(uint64_t timestamp_ns, HashedIndexMapping* remapping);

  void addActiveVertices();

  void updateAndAddArchivedFaces();

  void archiveBlockFaces(const BlockInfo& block_info,
                         RedunancyChecker& checker,
                         std::vector<Face>& pending_faces);

  void updateRemapping(MeshInterface& mesh, uint64_t timestamp_ns);

  void addPendingVertices(MeshDelta& delta, size_t start_index = 0);

 protected:
  double resolution_;
  double index_scale_;

  MeshDelta::Ptr delta_;
  MeshDelta::Ptr archive_delta_;

  std::vector<size_t> active_remapping_;
  BlockInfoMap block_info_map_;
  VoxelInfoMap vertices_map_;

  std::vector<VertexInfo> archived_vertices_;
  std::vector<Face> archived_faces_;

  uint16_t sequence_number_;
  size_t num_archived_vertices_;
  size_t num_archived_faces_;
};

}  // namespace kimera_pgmo
