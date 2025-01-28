/**
 * @file   mesh_compression.cpp
 * @brief  Mesh compression base class
 * @author Yun Chang
 */
#include "kimera_pgmo/compression/mesh_compression.h"

#include <pcl/conversions.h>

#include <algorithm>
#include <iterator>
#include <utility>

#include "kimera_pgmo/utils/common_functions.h"
#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {

/*void MeshCompression::compressAndIntegrate(*/
/*const pcl::PolygonMesh& input,*/
/*pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,*/
/*std::shared_ptr<std::vector<pcl::Vertices>> new_triangles,*/
/*std::shared_ptr<std::vector<size_t>> new_indices,*/
/*std::shared_ptr<std::unordered_map<size_t, size_t>> remapping,*/
/*const double& stamp_in_sec) {*/
/*// Extract vertices from input mesh*/
/*PointCloud input_vertices;*/
/*pcl::fromPCLPointCloud2(input.cloud, input_vertices);*/

/*compressAndIntegrate(input_vertices,*/
/*input.polygons,*/
/*new_vertices,*/
/*new_triangles,*/
/*new_indices,*/
/*remapping,*/
/*stamp_in_sec);*/
/*return;*/
/*}*/

/*
void MeshCompression::compressAndIntegrate(
  const pcl::PointCloud<pcl::PointXYZRGBA>& input_vertices,
  const std::vector<pcl::Vertices>& input_surfaces,
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
  std::shared_ptr<std::vector<pcl::Vertices>> new_triangles,
  std::shared_ptr<std::vector<size_t>> new_indices,
  std::shared_ptr<std::unordered_map<size_t, size_t>> remapping,
  const double& stamp_in_sec) {
// If there are no surfaces, return
if (input_vertices.size() < 3 || input_surfaces.size() == 0) {
  return;
}

// Avoid nullptr pointers
assert(nullptr != new_vertices);
assert(nullptr != new_triangles);
assert(nullptr != new_indices);
assert(nullptr != remapping);
new_vertices->clear();
new_triangles->clear();
new_indices->clear();

const size_t num_original_vertices = all_vertices_.size();

// Remaps from index in input vertices to index in all_vertices_
std::unordered_map<size_t, size_t> reindex;
// temporary reindex for all input vertices
std::vector<size_t> temp_reindex;

// Track possible new vertices
// Vector mapping index in temp_new_vertices to index in input vertices
std::vector<size_t> potential_new_vertices;
std::vector<bool> potential_new_vertices_check;

// Vertices that end up on each other after compression
std::unordered_map<size_t, std::vector<size_t>> converged_vertices;

// Temporary octree / cell for the points not stored
PointCloudXYZ::Ptr temp_new_vertices(new PointCloudXYZ);
initializeTempStructure(temp_new_vertices);
for (size_t i = 0; i < input_vertices.size(); i++) {
  const pcl::PointXYZRGBA& p = input_vertices.at(i);
  const pcl::PointXYZ p_xyz(p.x, p.y, p.z);
  int result_idx;
  // Check if vertex "seen" (or close to another seen vertex)
  if (checkIfVertexUnique(p_xyz, &result_idx)) {
    // New vertex will not be merged
    // Check if vertex in temp octree / cells
    if (checkIfVertexTempUnique(p_xyz, &result_idx)) {
      // New vertex not yet seen before
      temp_new_vertices->push_back(p_xyz);
      updateTempStructure(temp_new_vertices);
      potential_new_vertices.push_back(i);
      potential_new_vertices_check.push_back(false);
      temp_reindex.push_back(num_original_vertices + temp_new_vertices->size() - 1);
      converged_vertices.insert({i, std::vector<size_t>()});
    } else {
      // Add reindex index
      temp_reindex.push_back(num_original_vertices + result_idx);
      converged_vertices[potential_new_vertices[result_idx]].push_back(i);
    }
  } else {
    // This is a reobservation. Add to remap and new indices
    // Add reindex index
    reindex[i] = active_vertices_index_[result_idx];
    temp_reindex.push_back(active_vertices_index_[result_idx]);
    // Push to new indices if does not already yet
    if (!std::binary_search(new_indices->begin(),
                            new_indices->end(),
                            active_vertices_index_[result_idx])) {
      new_indices->insert(std::lower_bound(new_indices->begin(),
                                           new_indices->end(),
                                           active_vertices_index_[result_idx]),
                          active_vertices_index_[result_idx]);
    }
    // Update the last seen time of the vertex
    active_vertex_stamps_[result_idx] = stamp_in_sec;
  }
}
// First iteration through the faces to check the potential new vertices
for (const auto& s : input_surfaces) {
  pcl::Vertices reindex_s;
  bool has_new_vertex = false;
  for (size_t i : s.vertices) {
    if (temp_reindex.at(i) >= num_original_vertices) has_new_vertex = true;
    reindex_s.vertices.push_back(temp_reindex.at(i));
  }
  if (!has_new_vertex) continue;  // no need to check
  // Now check if new surface is acceptable
  if (reindex_s.vertices.size() < 3 ||
      reindex_s.vertices[0] == reindex_s.vertices[1] ||
      reindex_s.vertices[1] == reindex_s.vertices[2] ||
      reindex_s.vertices[2] == reindex_s.vertices[0])
    continue;  // degenerate
  // Passed degeneracy test so has at least one adjacent polygon. Pass check
  for (size_t i : reindex_s.vertices) {
    if (i >= num_original_vertices) {
      // This check is the main objective of this iteration through the faces
      potential_new_vertices_check[i - num_original_vertices] = true;
    }
  }
}
*remapping = reindex;
// Update reindex and the other structures
for (size_t i = 0; i < potential_new_vertices.size(); i++) {
  if (potential_new_vertices_check[i]) {
    // Passed check
    size_t input_idx = potential_new_vertices[i];
    pcl::PointXYZRGBA p = input_vertices.at(input_idx);
    pcl::PointXYZ p_xyz(p.x, p.y, p.z);
    // Add to octree / cells
    active_vertices_xyz_->push_back(p_xyz);
    updateStructure(active_vertices_xyz_);
    // Add to all vertices
    all_vertices_.push_back(p);
    all_vertex_stamps_.push_back(stampFromSec(stamp_in_sec));
    // Add to active vertices index
    active_vertices_index_.push_back(all_vertices_.size() - 1);
    active_vertex_stamps_.push_back(stamp_in_sec);
    // Upate reindex
    reindex[input_idx] = all_vertices_.size() - 1;
    remapping->insert(std::pair<size_t, size_t>{input_idx, all_vertices_.size() - 1});
    for (const auto& m : converged_vertices[input_idx]) {
      assert(temp_reindex[input_idx] == temp_reindex[m]);
      reindex[m] = all_vertices_.size() - 1;
      remapping->insert(std::pair<size_t, size_t>{m, all_vertices_.size() - 1});
    }
    // Add to new indices
    new_indices->push_back(all_vertices_.size() - 1);
    new_vertices->push_back(p);
  }
}

// Second iteration through the faces to add to new_triangles and update
// compressed mesh surfaces
for (auto s : input_surfaces) {
  pcl::Vertices reindex_s;
  bool new_surface = false;
  for (size_t idx : s.vertices) {
    // Check if reindex key exists, if not, already pruned earlier
    if (reindex.find(idx) == reindex.end()) break;
    reindex_s.vertices.push_back(reindex[idx]);
    if (reindex[idx] >= num_original_vertices) new_surface = true;
  }
  if (reindex_s.vertices.size() < 3) continue;

  // TODO: Check assumption that new surface cannot be constructed from
  // existing points. Topologically this makes sense.
  if (!new_surface) {
    new_surface = !SurfaceExists(reindex_s, adjacent_polygons_, polygons_);
  }
  if (!new_surface) continue;

  // Check if polygon has actual three diferent vertices
  // To avoid degeneracy
  if (reindex_s.vertices[0] == reindex_s.vertices[1] ||
      reindex_s.vertices[1] == reindex_s.vertices[2] ||
      reindex_s.vertices[2] == reindex_s.vertices[0])
    continue;

  // If it is a new surface, add
  if (new_surface) {
    // Definitely a new surface
    polygons_.push_back(reindex_s);
    new_triangles->push_back(reindex_s);
    // Update adjacent polygons
    for (size_t v : reindex_s.vertices) {
      adjacent_polygons_[v].push_back(polygons_.size() - 1);
    }
  }
}
return;
}
*/

void MeshCompression::compressAndIntegrate(const MeshInterface& mesh,
                                           PointCloud& new_vertices,
                                           std::vector<pcl::Vertices>& new_triangles,
                                           std::vector<size_t>& new_indices,
                                           HashedIndexMapping& remapping,
                                           double stamp_in_sec) {
  new_vertices.clear();
  new_triangles.clear();
  new_indices.clear();

  const size_t num_original_vertices = all_vertices_.size();

  // Remaps from index in input vertices to index in all_vertices_
  std::unordered_map<size_t, size_t> reindex;
  // temporary reindex for all new input vertices
  std::vector<size_t> temp_reindex;

  // Track possible new vertices
  // Vector mapping index in temp_new_vertices to index in input vertices
  std::vector<size_t> potential_new_vertices;
  std::vector<bool> potential_new_vertices_check;

  std::vector<pcl::Vertices> input_surfaces;

  PointCloudXYZ::Ptr temp_new_vertices(new PointCloudXYZ);
  initializeTempStructure(temp_new_vertices);

  size_t count = 0;
  // For book keeping track count to mesh block and index
  std::unordered_map<size_t, std::pair<BlockIndex, size_t>> count_to_block;
  PointCloud all_parsed_points;

  // Vertices that end up on each other after compression
  std::unordered_map<size_t, std::vector<size_t>> converged_vertices;

  // Iterate through the blocks
  for (const auto& block_index : mesh.blockIndices()) {
    mesh.markBlockActive(block_index);
    assert(mesh.activeBlockSize() % 3 == 0);
    // Add to remapping if not yet added previously
    remapping.insert({block_index, IndexMapping()});

    // Iterate through vertices of mesh block
    for (size_t i = 0; i < mesh.activeBlockSize(); ++i) {
      const pcl::PointXYZRGBA p = mesh.getActiveVertex(i);
      const pcl::PointXYZ p_xyz(p.x, p.y, p.z);
      // Book keep to track block index
      count_to_block[count] = {block_index, i};
      all_parsed_points.push_back(p);

      int result_idx = -1;
      // Check if vertex "seen" (or close to another seen vertex)
      if (checkIfVertexUnique(p_xyz, &result_idx)) {
        // New vertex will not be merged
        // Check if vertex in temp octree / cells
        if (checkIfVertexTempUnique(p_xyz, &result_idx)) {
          // New vertex not yet seen before
          temp_new_vertices->push_back(p_xyz);
          updateTempStructure(temp_new_vertices);
          potential_new_vertices.push_back(count);
          potential_new_vertices_check.push_back(false);
          temp_reindex.push_back(num_original_vertices + temp_new_vertices->size() - 1);
          converged_vertices.insert({count, std::vector<size_t>()});
        } else {
          // Add reindex index
          temp_reindex.push_back(num_original_vertices + result_idx);
          converged_vertices[potential_new_vertices[result_idx]].push_back(count);
        }
      } else {
        // This is a reobservation. Add to remap and new indices
        // Add reindex index
        reindex[count] = active_vertices_index_[result_idx];
        remapping.at(block_index)[i] = active_vertices_index_[result_idx];
        temp_reindex.push_back(active_vertices_index_[result_idx]);
        // Push to new indices if does not already yet
        if (!std::binary_search(new_indices.begin(),
                                new_indices.end(),
                                active_vertices_index_[result_idx])) {
          new_indices.insert(std::lower_bound(new_indices.begin(),
                                              new_indices.end(),
                                              active_vertices_index_[result_idx]),
                             active_vertices_index_[result_idx]);
        }
        // Update the last seen time of the vertex
        active_vertex_stamps_[result_idx] = stamp_in_sec;
      }

      // Every 3 vertices is a surface
      if (i % 3 == 2 && temp_reindex.at(count) != temp_reindex.at(count - 1)) {
        // Get the new indices of face
        size_t r_idx_0 = temp_reindex.at(count - 2);
        size_t r_idx_1 = temp_reindex.at(count - 1);
        size_t r_idx_2 = temp_reindex.at(count);

        // First check if there's a new vertex
        bool has_new_vertex =
            (r_idx_0 >= num_original_vertices || r_idx_1 >= num_original_vertices ||
             r_idx_2 >= num_original_vertices);

        if (!has_new_vertex) {
          count++;
          continue;  // no need to check
        }

        // Then check if face is degenerate
        bool degenerate =
            (r_idx_0 == r_idx_1 || r_idx_0 == r_idx_2 || r_idx_1 == r_idx_2);

        if (degenerate) {
          count++;
          continue;
        }

        // Add to input surfaces
        pcl::Vertices orig_s;
        orig_s.vertices.push_back(count - 2);
        orig_s.vertices.push_back(count - 1);
        orig_s.vertices.push_back(count);
        input_surfaces.push_back(orig_s);

        // Mark vertices as pass check (has at least one adjacent polygon)
        if (r_idx_0 >= num_original_vertices)
          potential_new_vertices_check[r_idx_0 - num_original_vertices] = true;
        if (r_idx_1 >= num_original_vertices)
          potential_new_vertices_check[r_idx_1 - num_original_vertices] = true;
        if (r_idx_2 >= num_original_vertices)
          potential_new_vertices_check[r_idx_2 - num_original_vertices] = true;
      }
      count++;
    }
  }

  // Update reindex and the other structures
  for (size_t i = 0; i < potential_new_vertices.size(); i++) {
    if (potential_new_vertices_check[i]) {
      // Passed check
      size_t input_idx = potential_new_vertices[i];
      pcl::PointXYZRGBA p = all_parsed_points.points[input_idx];
      pcl::PointXYZ p_xyz(p.x, p.y, p.z);
      // Add to octree / cells
      active_vertices_xyz_->push_back(p_xyz);
      updateStructure(active_vertices_xyz_);
      // Add to all vertices
      all_vertices_.push_back(p);
      all_vertex_stamps_.push_back(stampFromSec(stamp_in_sec));
      // Add to active vertices index
      active_vertices_index_.push_back(all_vertices_.size() - 1);
      active_vertex_stamps_.push_back(stamp_in_sec);
      // Upate reindex
      reindex[input_idx] = all_vertices_.size() - 1;
      remapping.at(count_to_block[input_idx].first)
          .insert({count_to_block[input_idx].second, all_vertices_.size() - 1});
      for (const auto& m : converged_vertices[input_idx]) {
        assert(temp_reindex[input_idx] == temp_reindex[m]);
        reindex[m] = all_vertices_.size() - 1;
        remapping.at(count_to_block[m].first)
            .insert({count_to_block[m].second, all_vertices_.size() - 1});
      }
      // Add to new indices
      new_indices.push_back(all_vertices_.size() - 1);
      new_vertices.push_back(p);
    }
  }

  // Second iteration through the faces to add to new_triangles and update
  // compressed mesh surfaces
  for (const auto& s : input_surfaces) {
    pcl::Vertices reindex_s;
    bool new_surface = false;
    for (size_t idx : s.vertices) {
      // Check if reindex key exists, if not, already pruned earlier
      if (reindex.find(idx) == reindex.end()) break;
      reindex_s.vertices.push_back(reindex[idx]);
      if (reindex[idx] >= num_original_vertices) new_surface = true;
    }
    if (reindex_s.vertices.size() < 3) continue;

    if (!new_surface) {
      // Check if surface exists already in mesh
      new_surface = !SurfaceExists(reindex_s, adjacent_polygons_, polygons_);
    }
    if (!new_surface) continue;

    // Check if polygon has actual three diferent vertices
    // To avoid degeneracy
    if (reindex_s.vertices[0] == reindex_s.vertices[1] ||
        reindex_s.vertices[1] == reindex_s.vertices[2] ||
        reindex_s.vertices[2] == reindex_s.vertices[0])
      continue;

    // If it is a new surface, add
    if (new_surface) {
      // Definitely a new surface
      polygons_.push_back(reindex_s);
      new_triangles.push_back(reindex_s);
      // Update adjacent polygons
      for (size_t v : reindex_s.vertices) {
        adjacent_polygons_[v].push_back(polygons_.size() - 1);
      }
    }
  }
}

void MeshCompression::pruneStoredMesh(double earliest_time_sec) {
  if (active_vertices_xyz_->size() == 0) {
    return;  // nothing to prune
  }

  // Entries in active_vertex_stamps_ shoudl correspond to number of points
  if (active_vertex_stamps_.size() != active_vertices_xyz_->size()) {
    SPARK_LOG(ERROR)
        << "Length of book-keeped vertex time does not match number of active points";
  }

  if (active_vertices_index_.size() != active_vertices_xyz_->size()) {
    SPARK_LOG(ERROR) << "Length of book-keeped vertex indices does not match number of "
                        "active points";
  }

  try {
    // Discard all vertices last detected before this time
    PointCloudXYZ temp_active_vertices;
    std::vector<double> temp_vertices_time;
    std::vector<size_t> temp_vertices_index;
    std::map<size_t, std::vector<size_t>> temp_adjacent_polygons;

    for (size_t i = 0; i < active_vertex_stamps_.size(); i++) {
      if (active_vertex_stamps_[i] > earliest_time_sec) {
        temp_active_vertices.push_back(active_vertices_xyz_->points[i]);
        temp_vertices_time.push_back(active_vertex_stamps_[i]);
        temp_vertices_index.push_back(active_vertices_index_[i]);
        temp_adjacent_polygons[active_vertices_index_[i]] =
            adjacent_polygons_[active_vertices_index_[i]];
      }
    }

    if (temp_active_vertices.size() < active_vertices_xyz_->size()) {
      active_vertices_xyz_->swap(temp_active_vertices);
      std::swap(active_vertex_stamps_, temp_vertices_time);
      std::swap(active_vertices_index_, temp_vertices_index);
      std::swap(adjacent_polygons_, temp_adjacent_polygons);

      // Reset structue
      reInitializeStructure(active_vertices_xyz_);
    }
  } catch (...) {
    SPARK_LOG(ERROR) << "MeshCompression: Failed to prune active mesh";
  }
}

}  // namespace kimera_pgmo
