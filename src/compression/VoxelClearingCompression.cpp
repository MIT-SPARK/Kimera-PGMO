/**
 * @file   VoxelClearingCompression.cpp
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#include <iterator>

#include "kimera_pgmo/compression/VoxelClearingCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/VoxbloxUtils.h"

namespace kimera_pgmo {

void updatePolygons(std::vector<pcl::Vertices> &faces,
                    const std::vector<size_t> &indices) {
  pcl::Vertices face;
  for (const auto &point_index : indices) {
    face.vertices.push_back(point_index);
    if (face.vertices.size() == 3) {
      // TODO(nathan) degenerate check
      faces.push_back(face);
      face.vertices.clear();
    }
  }
}

VoxelClearingCompression::VoxelClearingCompression(double resolution)
    : MeshCompression(resolution) {
  active_vertices_xyz_.reset(new PointCloudXYZ);
}

void VoxelClearingCompression::compressAndIntegrate(
    const voxblox_msgs::Mesh &mesh,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
    std::shared_ptr<std::vector<pcl::Vertices>> new_triangles,
    std::shared_ptr<std::vector<size_t>> new_indices,
    std::shared_ptr<VoxbloxIndexMapping> remapping,
    const double &stamp_in_sec) {
  // Avoid nullptr pointers
  assert(nullptr != new_vertices);
  assert(nullptr != new_triangles);
  assert(nullptr != new_indices);
  assert(nullptr != remapping);
  new_vertices->clear();
  new_triangles->clear();
  new_indices->clear();

  updateRemapping(mesh, stamp_in_sec, remapping);
  updateVertices();
  updateActiveIndices();
}

void VoxelClearingCompression::updateActiveIndices() {
  active_vertices_index_.clear();
  active_vertices_index_.reserve(indices_to_active_refs_.size());
  for (const auto &id_count_pair : indices_to_active_refs_) {
    active_vertices_index_.push_back(id_count_pair.first);
  }
}

void VoxelClearingCompression::pruneStoredMesh(const double &earliest_time_s) {
  BlockIndexList to_clear;
  for (const auto &idx_time_pair : block_update_times_) {
    if (idx_time_pair.second <= earliest_time_s) {
      to_clear.push_back(idx_time_pair.first);
    }
  }

  pruneMeshBlocks(to_clear);
}

void VoxelClearingCompression::clearArchivedBlocks(const voxblox_msgs::Mesh& mesh) {
  BlockIndexList to_clear;
  for (const auto& block : mesh.mesh_blocks) {
    BlockIndex block_index(block.index[0], block.index[1], block.index[2]);
    to_clear.push_back(block_index);
  }

  pruneMeshBlocks(to_clear);
}

void VoxelClearingCompression::pruneMeshBlocks(const BlockIndexList& to_clear) {
  for (const auto &idx : to_clear) {
    auto voxels = prev_meshes_[idx];
    for (const auto &voxel : voxels) {
      const size_t mesh_idx = vertices_map_[voxel];
      indices_to_active_refs_[mesh_idx]--; // only decrement active ref count

      if (indices_to_active_refs_[mesh_idx] != 0) {
        continue;
      }

      // on archive (i.e. no more active blocks contain the vertex), we delete
      // the ref counts and then erase the voxel -> vertex association to
      // allow revisiting to make a new vertex
      indices_to_active_refs_.erase(mesh_idx);
      indices_to_inactive_refs_.erase(mesh_idx);
      vertices_map_.erase(voxel);
    }

    polygons_.resize(archived_polygon_size_);
    updatePolygons(polygons_, block_face_map_[idx]);
    archived_polygon_size_ = polygons_.size();

    block_face_map_.erase(idx);
    prev_meshes_.erase(idx);
    block_update_times_.erase(idx);
  }

  updateActiveIndices();
}

void VoxelClearingCompression::updateVertices() {
  polygons_.resize(archived_polygon_size_);

  for (const auto &idx_indices_pair : block_face_map_) {
    updatePolygons(polygons_, idx_indices_pair.second);
  }
}

void VoxelClearingCompression::updateRemapping(
    const voxblox_msgs::Mesh& mesh,
    double stamp_in_sec,
    std::shared_ptr<VoxbloxIndexMapping> remapping) {
  const auto threshold_inv = 1.0 / resolution_;
  const auto block_edge_length = mesh.block_edge_length;

  const ros::Time ros_stamp(stamp_in_sec);

  pcl::PointXYZRGBA fake_point;
  fake_point.x = 0.0f;
  fake_point.y = 0.0f;
  fake_point.z = 0.0f;

  for (const auto &block : mesh.mesh_blocks) {
    BlockIndex block_index(block.index[0], block.index[1], block.index[2]);
    block_update_times_[block_index] = stamp_in_sec;
    remapping->insert(VoxbloxIndexPair(block_index, IndexMapping()));

    const size_t block_size = block.x.size();
    std::vector<size_t> face_map;
    face_map.reserve(block_size);

    voxblox::LongIndexSet curr_voxels;
    for (size_t i = 0; i < block_size; ++i) {
      const pcl::PointXYZRGBA p = ExtractPoint(block, block_edge_length, i);
      const voxblox::LongIndex vertex_index(std::round(p.x * threshold_inv),
                                            std::round(p.y * threshold_inv),
                                            std::round(p.z * threshold_inv));

      size_t mesh_index;
      auto map_iter = vertices_map_.find(vertex_index);
      if (map_iter != vertices_map_.end()) {
        mesh_index = map_iter->second;

      } else {
        mesh_index = max_index_;
        ++max_index_;

        all_vertex_stamps_.push_back(ros_stamp);
        vertices_map_[vertex_index] = mesh_index;
        indices_to_active_refs_[mesh_index] = 0;
        indices_to_inactive_refs_[mesh_index] = 0;
        all_vertices_.push_back(p);
      }

      remapping->at(block_index)[i] = mesh_index;
      all_vertices_[mesh_index] = p;
      face_map.push_back(mesh_index);
      if (!curr_voxels.count(vertex_index)) {
        // increment ref count, even if the block has already been counted.
        // We handle decrementing it later
        ++indices_to_active_refs_[mesh_index];
        ++indices_to_inactive_refs_[mesh_index];
      }

      curr_voxels.insert(vertex_index);
    }

    block_face_map_[block_index] = face_map;

    auto prev_mesh = prev_meshes_.find(block_index);
    if (prev_mesh == prev_meshes_.end()) {
      prev_meshes_[block_index] = curr_voxels;
      continue;
    }

    for (const auto &prev : prev_mesh->second) {
      const size_t mesh_index = vertices_map_.at(prev);
      // we want to decrement every previous vertex, as they would have been
      // incremental for this block in the previous loop even though they
      // weren't new. When there was no current vertex that mapped to this
      // index, this actually decreases the ref count (instead of just
      // maintaining it
      --indices_to_active_refs_[mesh_index];
      --indices_to_inactive_refs_[mesh_index];

      if (indices_to_inactive_refs_[mesh_index] == 0) {
        // no blocks point to the mesh point any more, so delete
        vertices_map_.erase(prev);
        indices_to_active_refs_.erase(mesh_index);
        indices_to_inactive_refs_.erase(mesh_index);
        all_vertices_[mesh_index] = fake_point;
        empty_slots_.push_back(mesh_index);
        continue;
      }

      if (indices_to_active_refs_[mesh_index] != 0) {
        // we have active references to the mesh vertex, so continue tracking
        continue;
      }

      // on archive (i.e. no more active blocks contain the vertex), we delete
      // the ref counts and then erase the voxel -> vertex association to allow
      // revisiting to make a new vertex
      indices_to_active_refs_.erase(mesh_index);
      indices_to_inactive_refs_.erase(mesh_index);
      vertices_map_.erase(prev);
    }

    prev_meshes_[block_index] = curr_voxels;
  }
}

} // namespace kimera_pgmo
