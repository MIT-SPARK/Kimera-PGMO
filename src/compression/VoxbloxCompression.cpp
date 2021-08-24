/**
 * @file   VoxbloxCompression.cpp
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 */
#include <iterator>

#include <ros/ros.h>

#include "kimera_pgmo/compression/VoxbloxCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace voxblox = vxb;

namespace kimera_pgmo {

VoxbloxCompression::VoxbloxCompression(double resolution)
    : resolution_(resolution) {
  active_vertices_xyz_.reset(new PointCloudXYZ);
}

VoxbloxCompression::~VoxbloxCompression() {}

void VoxbloxCompression::compressAndIntegrate(
    const pcl::PolygonMesh& input,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
    boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles,
    boost::shared_ptr<std::vector<size_t> > new_indices,
    boost::shared_ptr<std::unordered_map<size_t, size_t> > remapping,
    const double& stamp_in_sec) {
  // Extract vertices from input mesh
  PointCloud input_vertices;
  pcl::fromPCLPointCloud2(input.cloud, input_vertices);

  compressAndIntegrate(input_vertices,
                       input.polygons,
                       new_vertices,
                       new_triangles,
                       new_indices,
                       remapping,
                       stamp_in_sec);
  return;
}

void VoxbloxCompression::compressAndIntegrate(
    const pcl::PointCloud<pcl::PointXYZRGBA>& input_vertices,
    const std::vector<pcl::Vertices>& input_surfaces,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
    boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles,
    boost::shared_ptr<std::vector<size_t> > new_indices,
    boost::shared_ptr<std::unordered_map<size_t, size_t> > remapping,
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

  const size_t num_original_active_vertices = active_vertices_xyz_->size();
  const size_t num_original_vertices = all_vertices_.size();

  // Remaps from index in input vertices to index in all_vertices_
  std::unordered_map<size_t, size_t> reindex;
  // temporary reindex for all input vertices
  std::vector<size_t> temp_reindex;

  // Track possible new vertices
  // Vector mapping index in temp_new_vertices to index in input vertices
  std::vector<size_t> potential_new_vertices;
  std::vector<bool> potential_new_vertices_check;

  // Temporary cell hash for the points not in stored cell hash
  PointCloudXYZ::Ptr temp_new_vertices(new PointCloudXYZ);
  voxblx::LongIndexHashMapType<size_t>::type temp_cell_hash;
  const double threshold_inv = 1. / resolution_;

  for (size_t i = 0; i < input_vertices.size(); i++) {
    const pcl::PointXYZRGBA& p = input_vertices.at(i);
    const pcl::PointXYZ p_xyz(p.x, p.y, p.z);
    vxb::Point vertex;
    vertex << p.x, p.y, p.z;
    const Eigen::Vector3d scaled_vector = vertex.cast<double>() * threshold_inv;
    const vxb::LongIndex vertex_3D_index =
        vxb::LongIndex(std::round(scaled_vector.x()),
                       std::round(scaled_vector.y()),
                       std::round(scaled_vector.z()));

    vxb::LongIndexHashMapType<size_t>::type::const_iterator it =
        cell_hash_.find(vertex_3D_index);

    // Check if in cell hash
    if (it == cell_hash_.end()) {
      // New vertex not in cell hash
      // Check if in new cell hash
      vxb::LongIndexHashMapType<size_t>::type::const_iterator temp_it =
          temp_cell_hash.find(vertex_3D_index);
      if (temp_it == temp_cell_hash.end()) {
        // New vertex not in temp cell hash
        temp_new_vertices->push_back(p_xyz);
        temp_cell_hash.emplace(vertex_3D_index, temp_new_vertices->size() - 1);
        potential_new_vertices.push_back(i);
        potential_new_vertices_check.push_back(false);
        temp_reindex.push_back(num_original_vertices +
                               temp_new_vertices->size() - 1);
      } else {
        float unused = 0.f;
        int result_idx = temp_it->second;
        // Add reindex index
        temp_reindex.push_back(num_original_vertices + result_idx);
      }
    } else {
      // Vertex in cell hash. This is a reobservation. Add to remap and new
      // indices
      int result_idx = it->second;
      // Add reindex index
      reindex[i] = active_vertices_index_[result_idx];
      temp_reindex.push_back(active_vertices_index_[result_idx]);
      // Push to new indices if does not already yet
      if (std::find(new_indices->begin(),
                    new_indices->end(),
                    active_vertices_index_[result_idx]) == new_indices->end())
        new_indices->push_back(active_vertices_index_[result_idx]);
      // Update the last seen time of the vertex
      vertices_latest_time_[result_idx] = stamp_in_sec;
    }
  }
  // First iteration through the faces to check the potential new vertices
  for (auto s : input_surfaces) {
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
      vxb::Point vertex;
      vertex << p.x, p.y, p.z;
      const Eigen::Vector3d scaled_vector =
          vertex.cast<double>() * threshold_inv;
      const vxb::LongIndex vertex_3D_index =
          vxb::LongIndex(std::round(scaled_vector.x()),
                         std::round(scaled_vector.y()),
                         std::round(scaled_vector.z()));

      // Add to stored cell hash
      active_vertices_xyz_->push_back(p_xyz);
      cell_hash_.emplace(vertex_3D_index, active_vertices_xyz_->size() - 1);
      // Add to all vertices
      all_vertices_.push_back(p);
      // Add to active vertices index
      active_vertices_index_.push_back(all_vertices_.size() - 1);
      vertices_latest_time_.push_back(stamp_in_sec);
      // Upate reindex
      reindex[potential_new_vertices[i]] = all_vertices_.size() - 1;
      remapping->insert(std::pair<size_t, size_t>{potential_new_vertices[i],
                                                  all_vertices_.size() - 1});
      for (size_t m = 0; m < temp_reindex.size(); m++) {
        if (potential_new_vertices[i] != m &&
            temp_reindex[i] == temp_reindex[m]) {
          remapping->insert(
              std::pair<size_t, size_t>{m, all_vertices_.size() - 1});
        }
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
      if (reindex.find(idx) == reindex.end()) continue;
      reindex_s.vertices.push_back(reindex[idx]);
      if (reindex[idx] >= num_original_vertices) new_surface = true;
    }
    if (reindex_s.vertices.size() < 3) continue;

    // Check if polygon has actual three diferent vertices
    // To avoid degeneracy
    if (reindex_s.vertices[0] == reindex_s.vertices[1] ||
        reindex_s.vertices[1] == reindex_s.vertices[2] ||
        reindex_s.vertices[2] == reindex_s.vertices[0])
      continue;

    // Check if it is a new surface constructed from existing points
    if (!new_surface) {
      new_surface = !SurfaceExists(reindex_s, adjacent_polygons_, polygons_);
    }

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

void VoxbloxCompression::pruneStoredMesh(const double& earliest_time_sec) {
  if (active_vertices_xyz_->size() == 0) return;  // nothing to prune
  // Entries in vertices_latest_time_ shoudl correspond to number of points
  if (vertices_latest_time_.size() != active_vertices_xyz_->size()) {
    ROS_ERROR(
        "Length of book-keeped vertex time does not match number of active "
        "points. ");
  }

  if (active_vertices_index_.size() != active_vertices_xyz_->size()) {
    ROS_ERROR(
        "Length of book-keeped vertex indices does not match number of "
        "active "
        "points. ");
  }

  try {
    // Discard all vertices last detected before this time
    PointCloudXYZ temp_active_vertices;
    std::vector<double> temp_vertices_time;
    std::vector<size_t> temp_vertices_index;
    std::map<size_t, std::vector<size_t> > temp_adjacent_polygons;

    for (size_t i = 0; i < vertices_latest_time_.size(); i++) {
      if (vertices_latest_time_[i] > earliest_time_sec) {
        temp_active_vertices.push_back(active_vertices_xyz_->points[i]);
        temp_vertices_time.push_back(vertices_latest_time_[i]);
        temp_vertices_index.push_back(active_vertices_index_[i]);
        temp_adjacent_polygons[active_vertices_index_[i]] =
            adjacent_polygons_[active_vertices_index_[i]];
      }
    }

    if (temp_active_vertices.size() < active_vertices_xyz_->size()) {
      active_vertices_xyz_->swap(temp_active_vertices);
      std::swap(vertices_latest_time_, temp_vertices_time);
      std::swap(active_vertices_index_, temp_vertices_index);
      std::swap(adjacent_polygons_, temp_adjacent_polygons);

      // Reset cell hash
      cell_hash_.clear();
      idx = 0;
      for (const auto& p : active_vertices_xyz_->points) {
        vxb::Point vertex;
        // TODO(Yun write a function for this)
        vertex << p.x, p.y, p.z;
        const Eigen::Vector3d scaled_vector =
            vertex.cast<double>() * threshold_inv;
        const vxb::LongIndex vertex_3D_index =
            vxb::LongIndex(std::round(scaled_vector.x()),
                           std::round(scaled_vector.y()),
                           std::round(scaled_vector.z()));
        cell_hash_.emplace(vertex_3D_index, idx);
        idx++;
      }
    }
  } catch (...) {
    ROS_ERROR("VoxbloxCompression: Failed to prune active mesh. ");
  }
  return;
}

}  // namespace kimera_pgmo