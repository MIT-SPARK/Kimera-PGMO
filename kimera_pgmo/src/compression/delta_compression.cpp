/**
 * @file   delta_compression.cpp
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include "kimera_pgmo/compression/delta_compression.h"

#include <iterator>

#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

using spatial_hash::BlockIndex;
using spatial_hash::BlockIndices;

struct RedunancyChecker {
  using SparseAdjacencyMatrix = std::map<size_t, std::set<size_t>>;

  bool check(const Face& face) const {
    const bool present = hasEdge(face.v1, face.v2) && hasEdge(face.v2, face.v3) &&
                         hasEdge(face.v2, face.v3);
    return !present;
  }

  void add(const Face& face) {
    addEdge(face.v1, face.v2);
    addEdge(face.v2, face.v3);
    addEdge(face.v3, face.v1);
  }

  bool hasEdge(size_t source, size_t target) const {
    const auto iter = A.find(source);
    if (iter == A.end()) {
      return false;
    }

    return iter->second.count(target);
  }

  void addEdge(size_t source, size_t target) {
    auto iter = A.find(source);
    if (iter == A.end()) {
      iter = A.insert({source, {}}).first;
    }

    iter->second.insert(target);
  }

  SparseAdjacencyMatrix A;
};

void VertexInfo::addObservation() const {
  ++active_refs;
  ++inactive_refs;
}

void VertexInfo::archiveObservation() const { --active_refs; }

void VertexInfo::removeObservation() const {
  --active_refs;
  --inactive_refs;
}

bool VertexInfo::notObserved() const { return inactive_refs <= 0; }

bool VertexInfo::shouldArchive() const { return active_refs <= 0; }

DeltaCompression::DeltaCompression(double resolution)
    : resolution_(resolution),
      index_scale_(1.0 / resolution),
      num_archived_vertices_(0),
      num_archived_faces_(0) {}

void DeltaCompression::addPoint(const pcl::PointXYZRGBA& point,
                                std::optional<uint32_t> semantic_label,
                                uint64_t timestamp_ns,
                                std::vector<size_t>& face_map,
                                spatial_hash::LongIndexSet& curr_voxels) {
  // do voxel hashing at compression size to determine remapping to previous compressed
  // vertex (if it exists)
  const spatial_hash::LongIndex vertex_index(std::round(point.x * index_scale_),
                                        std::round(point.y * index_scale_),
                                        std::round(point.z * index_scale_));

  auto info_iter = vertices_map_.find(vertex_index);
  if (info_iter == vertices_map_.end()) {
    info_iter = vertices_map_.insert({vertex_index, {}}).first;
    // force update for a new point later (timestamp gets overwritten with correct
    // timestamp)
    info_iter->second.timestamp_ns = timestamp_ns + 1;
  }

  auto& info = info_iter->second;
  if (info.timestamp_ns != timestamp_ns) {
    info.timestamp_ns = timestamp_ns;
    info.point = point;
    info.label = semantic_label;
    const size_t prev_index = info.mesh_index;
    info.mesh_index = active_remapping_.size();
    active_remapping_.push_back(prev_index);  // cache previous index
  }
  face_map.push_back(info.mesh_index);
  if (!curr_voxels.count(vertex_index)) {
    info.addObservation();  // add one observation per block
    curr_voxels.insert(vertex_index);
  }
}

void DeltaCompression::removeBlockObservations(const BlockIndex& /* block_index */,
                                               const spatial_hash::LongIndexSet& to_remove) {
  // for every previous voxel observed by the block
  //   - remove an observation
  //   - if there are NO "inactive" (archived) blocks pointing to the voxel
  //     - register the voxed as deleted with the mesh delta
  //     - erase the voxel from the active voxel map
  //   - if there are other blocks still pointing to the voxel, do nothing
  //   - if there are "inactive" (archived) blocks pointing to the voxel
  //     - archive the voxel in the mesh delta and add the new index to the current
  //       remapping
  //     - erase the voxel from the active map
  for (const auto& prev : to_remove) {
    const auto& info = vertices_map_.at(prev);
    info.removeObservation();

    if (info.notObserved()) {  // inactive_refs <= 0
      delta_->deleted_indices.insert(info.mesh_index);
      vertices_map_.erase(prev);
      continue;
    }

    if (!info.shouldArchive()) {  // active_refs > 0
      continue;                   // nothing else to do
    }

    // we can't observe a vertex and then need to archive it in the same pass, so
    // info.mesh_index should point to the previous index
    const size_t mesh_index =
        delta_->addVertex(info.timestamp_ns, info.point, info.label, true);
    delta_->prev_to_curr[info.mesh_index] = mesh_index;
    vertices_map_.erase(prev);
  }
}

void DeltaCompression::addActiveVertices(uint64_t timestamp_ns) {
  for (auto& id_info_pair : vertices_map_) {
    // vertices here are guaranteed to be unique
    auto& info = id_info_pair.second;
    const size_t mesh_index =
        delta_->addVertex(info.timestamp_ns, info.point, info.label);

    if (info.timestamp_ns == timestamp_ns) {
      if (!info.is_new) {
        // we use the active remapping slot as temporary variable to cache
        // the index in the previous delta
        const size_t prev_mesh_index = active_remapping_[info.mesh_index];
        delta_->prev_to_curr[prev_mesh_index] = mesh_index;
      } else {
        delta_->new_indices.insert(mesh_index);
        info.is_new = false;
      }
      delta_->observed_indices.insert(mesh_index);

      // set to correct delta index for face construction
      active_remapping_[info.mesh_index] = mesh_index;
    } else {
      // if we haven't seen this vertex in this pass, add to prev_to_curr map
      delta_->prev_to_curr[info.mesh_index] = mesh_index;
    }

    // set mesh index to point to the correct index in the current delta
    info.mesh_index = mesh_index;
  }
}

void DeltaCompression::addActiveFaces(uint64_t timestamp_ns,
                                      HashedIndexMapping* remapping) {
  // for every current tracked block we
  //   - iterate through every "face" (set of 3 vertex indices)
  //   - if the face is from a block contained in the latest message, grab the new face
  //     indices from the remapping between the latest indices and their positions in
  //     the newest delta. Otherwise, carry any changes in indices between the last mesh
  //     delta and the newest one forward
  //  - store the remapping between every original vertex index in the latest message
  //    and the compressed vertex index in the latest delta (as the remapping is now
  //    fixed at this point, but was not when active vertices were being added)
  //  - add the face if it isn't degenerate or redundant
  const auto& prev_to_curr = delta_->prev_to_curr;
  // note that we only need to check for duplicates per each "type" of face
  RedunancyChecker checker;
  for (auto& id_info_pair : block_info_map_) {
    auto& block_info = id_info_pair.second;
    const bool was_updated = block_info.update_time == timestamp_ns;

    IndexMapping* block_remap = nullptr;
    if (remapping && was_updated) {
      block_remap = &(remapping->insert({id_info_pair.first, {}}).first->second);
    }

    auto& indices = block_info.indices;
    for (size_t i = 0; i < indices.size(); i += 3) {
      if (was_updated) {
        // face points to active vertices, so grab actual delta index and add to
        // remapping
        indices[i] = active_remapping_[indices[i]];
        indices[i + 1] = active_remapping_[indices[i + 1]];
        indices[i + 2] = active_remapping_[indices[i + 2]];
      } else {
        indices[i] = prev_to_curr.at(indices[i]);
        indices[i + 1] = prev_to_curr.at(indices[i + 1]);
        indices[i + 2] = prev_to_curr.at(indices[i + 2]);
      }

      if (block_remap) {
        // block_remap is only valid if remapping is provided and the block was updated
        // from this compression pass
        block_remap->insert({i, indices[i]});
        block_remap->insert({i + 1, indices[i + 1]});
        block_remap->insert({i + 2, indices[i + 2]});
      }

      const Face face(indices, i);
      if (!face.valid()) {
        continue;
      }

      if (!checker.check(face)) {
        continue;
      }

      checker.add(face);
      delta_->addFace(face);
    }
  }
}

MeshDelta::Ptr DeltaCompression::update(MeshInterface& mesh,
                                        uint64_t timestamp_ns,
                                        HashedIndexMapping* remapping) {
  while (timestamp_cache_.count(timestamp_ns)) {
    ++timestamp_ns;
  }
  timestamp_cache_.insert(timestamp_ns);
  // 1) update happens independently of calls to clearArchivedBlocks (and
  // pruneStoredMesh, which calls clearArchivedBlocks). we cache the results of the
  // archive to a separate mesh delta and copy them over
  if (archive_delta_) {
    delta_ = archive_delta_;
    archive_delta_.reset();
  } else {
    delta_.reset(new MeshDelta(num_archived_vertices_, num_archived_faces_));
  }

  // 2) fully archive any face that belongs to a block that has been previously archived
  // that only points to archived vertices, and queue any partially archived face.
  archiveBlockFaces();

  // 3) compute the actual compression of the latest mesh, determining the remapping
  // between vertex indices in the provided mesh to vertices in the compressed mesh
  updateRemapping(mesh, timestamp_ns);

  // 4) Add all active vertices and faces to the output mesh delta. This means any
  // compressed vertex that is the result of one or more block vertices that hasn't
  // been archived and any face from any block that hasn't been archived
  addActive(timestamp_ns, remapping);

  // 5) Update and add all archive faces with the new remapping. This remaps previously
  // partially archived faces with any newly archived vertices and also archives them in
  // the delta if they become fully archived. Note that they cannot be duplicates of any
  // faces added in the next for-loop, as they must point to at least one vertex that
  // was archived before this pass that no other active face points to (active faces
  // cannot point at archived vertices)
  updateAndAddArchivedFaces();

  // 6) Finalize mesh delta book-keeping and return the new mesh update
  num_archived_vertices_ = delta_->getTotalArchivedVertices();
  num_archived_faces_ = delta_->getTotalArchivedFaces();
  return delta_;
}

void DeltaCompression::updateRemapping(MeshInterface& mesh, uint64_t stamp_ns) {
  // For every block in the latest input mesh:
  //   - initialize book-keeping for previously unobserved blocks
  //   - add all vertices from the block into the compression (this increases ref
  //   counts)
  //   - remove any previous observations from the block if the block isn't new (this
  //     decreases ref counts to be correct)
  active_remapping_.clear();
  for (const auto& block_index : mesh.blockIndices()) {
    bool is_block_new = false;
    auto block_iter = block_info_map_.find(block_index);
    if (block_iter == block_info_map_.end()) {
      is_block_new = true;
      block_iter = block_info_map_.insert({block_index, {{}, stamp_ns, {}}}).first;
    }

    auto& block_info = block_iter->second;
    block_info.update_time = stamp_ns;
    block_info.indices.clear();

    spatial_hash::LongIndexSet curr_voxels;
    mesh.markBlockActive(block_index);
    for (size_t i = 0; i < mesh.activeBlockSize(); ++i) {
      addPoint(mesh.getActiveVertex(i),
               mesh.hasSemantics() ? mesh.getActiveSemantics(i) : std::nullopt,
               stamp_ns,
               block_info.indices,
               curr_voxels);
    }

    if (!is_block_new) {
      removeBlockObservations(block_index, block_info.vertices);
    }

    block_info.vertices = curr_voxels;
  }
}

void DeltaCompression::addActive(uint64_t stamp_ns, HashedIndexMapping* remapping) {
  addActiveVertices(stamp_ns);
  // note: this provides an invariant: every face that belongs to a block in
  // block_info_map_ has the correct indices after this is called and before the next
  // updateRemapping is called. This means that anyone archiving blocks can directly use
  // the faces without doing any remapping
  addActiveFaces(stamp_ns, remapping);
}

void DeltaCompression::pruneStoredMesh(uint64_t earliest_time_ns) {
  BlockIndices to_clear;
  for (const auto& id_info_pair : block_info_map_) {
    if (id_info_pair.second.update_time <= earliest_time_ns) {
      to_clear.push_back(id_info_pair.first);
    }
  }

  pruneMeshBlocks(to_clear);
}

void DeltaCompression::clearArchivedBlocks(const BlockIndices& to_clear) {
  pruneMeshBlocks(to_clear);
}

void DeltaCompression::pruneMeshBlocks(const BlockIndices& to_clear) {
  archive_delta_.reset(new MeshDelta(num_archived_vertices_, num_archived_faces_));

  // for every block we want to archive:
  //   - update ref counts for any vertices the block contains
  //   - any voxel/vertex that only has archived blocks pointing to it should get
  //     archived by being added to the archive_delta_ (to be used as a starting point
  //     next update call)
  for (const auto& idx : to_clear) {
    auto block_iter = block_info_map_.find(idx);
    if (block_iter == block_info_map_.end()) {
      // TODO(nathan) warn about pruning unknown block
      continue;
    }

    archived_block_info_map_.insert({idx, block_iter->second});
    const auto& block_info = block_iter->second;
    for (const auto& voxel : block_info.vertices) {
      auto& info = vertices_map_[voxel];
      info.archiveObservation();

      if (!info.shouldArchive()) {
        continue;
      }

      const size_t new_index =
          archive_delta_->addVertex(info.timestamp_ns, info.point, info.label, true);
      archive_delta_->prev_to_curr[info.mesh_index] = new_index;
      vertices_map_.erase(voxel);
    }

    block_info_map_.erase(idx);
  }
}

size_t getRemappedIndex(const std::map<size_t, size_t>& remapping, size_t original) {
  const auto iter = remapping.find(original);
  // TODO(Yun): technically should always have key in remapping
  return iter == remapping.end() ? original : iter->second;
}

size_t getRemappedIndex(const std::map<size_t, size_t>& remapping,
                        size_t original,
                        size_t threshold) {
  if (original < threshold) {
    return original;
  }

  return getRemappedIndex(remapping, original);
}

void DeltaCompression::updateAndAddArchivedFaces() {
  // note that we only need to check for duplicates per each "type" of face
  RedunancyChecker checker;
  const auto& prev_to_curr = delta_->prev_to_curr;
  const auto archive_threshold = delta_->getTotalArchivedVertices();

  auto iter = archived_faces_.begin();
  while (iter != archived_faces_.end()) {
    auto& face = *iter;
    face.v1 = getRemappedIndex(prev_to_curr, face.v1, num_archived_vertices_);
    face.v2 = getRemappedIndex(prev_to_curr, face.v2, num_archived_vertices_);
    face.v3 = getRemappedIndex(prev_to_curr, face.v3, num_archived_vertices_);

    if (!face.valid()) {
      iter = archived_faces_.erase(iter);
      continue;
    }

    if (!checker.check(face)) {
      // if we hit this point, there is another face pending archive that contains the
      // same information. Regardless of whether or not the face is going to be archived
      // this pass, we should always drop the duplicates as the face will already be
      // covered
      iter = archived_faces_.erase(iter);
      continue;
    }

    const bool can_archive = canBeArchived(face, archive_threshold);
    checker.add(face);
    delta_->addFace(face, can_archive);
    if (can_archive) {
      iter = archived_faces_.erase(iter);
    } else {
      ++iter;
    }
  }
}

bool DeltaCompression::canBeArchived(const Face& face, size_t archive_threshold) const {
  return face.v1 < archive_threshold && face.v2 < archive_threshold &&
         face.v3 < archive_threshold;
}

bool DeltaCompression::canBeArchived(const Face& face) const {
  return canBeArchived(face, num_archived_vertices_);
}

void DeltaCompression::archiveBlockFaces() {
  // this remapping points from the previous active index to the current archival index
  // of all vertices archived from the block being archived. Any active vertex (or
  // previously archived vertex) will not be in the remapping
  const auto archive_threshold = delta_->getTotalArchivedVertices();
  const auto& prev_to_curr = delta_->prev_to_curr;
  // note that we only need to check for duplicates per each "type" of face
  RedunancyChecker checker;
  for (const auto& idx_info_pair : archived_block_info_map_) {
    const auto& idx = idx_info_pair.first;
    // for all faces in the block:
    //   - if the face has some indices that haven't been archived yet, store for later
    //     archival
    //   - remap all archived vertices to their correct indices and archive the face
    const auto& indices = archived_block_info_map_.at(idx).indices;
    for (size_t i = 0; i < indices.size(); i += 3) {
      Face face(indices.at(i), indices.at(i + 1), indices.at(i + 2));
      bool v1_can_archive =
          prev_to_curr.count(face.v1) || face.v1 < num_archived_vertices_;
      bool v2_can_archive =
          prev_to_curr.count(face.v2) || face.v2 < num_archived_vertices_;
      bool v3_can_archive =
          prev_to_curr.count(face.v3) || face.v3 < num_archived_vertices_;
      if (!v1_can_archive || !v2_can_archive || !v3_can_archive) {
        // push any face that we can't deal with currently to be considered for archival
        // later. Crucially, we don't remap any face indices here as we can't
        // distinguish whether an index was remapped here or not
        archived_faces_.push_back(face);
        continue;
      }

      face.v1 = getRemappedIndex(prev_to_curr, face.v1);
      face.v2 = getRemappedIndex(prev_to_curr, face.v2);
      face.v3 = getRemappedIndex(prev_to_curr, face.v3);
      if (!face.valid()) {
        continue;
      }

      if (!canBeArchived(face, archive_threshold)) {
        archived_faces_.push_back(face);
        continue;
      }

      if (!checker.check(face)) {
        continue;
      }

      checker.add(face);
      delta_->addFace(face, true);
    }
  }

  archived_block_info_map_.clear();
}

}  // namespace kimera_pgmo
