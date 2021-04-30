/**
 * @file   OctreeCompression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <algorithm>
#include <iterator>
#include <map>
#include <memory>
#include <utility>

#include <ros/ros.h>

#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {

OctreeCompression::OctreeCompression(double resolution)
    : octree_resolution_(resolution) {
  active_vertices_xyz_.reset(new PointCloudXYZ);
  // Initialize octree
  octree_.reset(new Octree(resolution));
  octree_->setInputCloud(active_vertices_xyz_);
}

OctreeCompression::~OctreeCompression() {}

void OctreeCompression::compressAndIntegrate(
    const pcl::PolygonMesh& input,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
    boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles,
    boost::shared_ptr<std::vector<size_t> > new_indices,
    const double& stamp_in_sec) {
  // Extract vertices from input mesh
  PointCloud input_vertices;
  pcl::fromPCLPointCloud2(input.cloud, input_vertices);

  compressAndIntegrate(input_vertices,
                       input.polygons,
                       new_vertices,
                       new_triangles,
                       new_indices,
                       stamp_in_sec);
  return;
}

void OctreeCompression::compressAndIntegrate(
    const pcl::PointCloud<pcl::PointXYZRGBA>& input_vertices,
    const std::vector<pcl::Vertices>& input_surfaces,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
    boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles,
    boost::shared_ptr<std::vector<size_t> > new_indices,
    const double& stamp_in_sec) {
  // If there are no surfaces, return
  if (input_vertices.size() < 3 || input_surfaces.size() == 0) {
    return;
  }

  // Avoid nullptr pointers
  assert(nullptr != new_vertices);
  assert(nullptr != new_triangles);
  assert(nullptr != new_indices);

  // Place vertices through octree for compression
  double min_x, min_y, min_z, max_x, max_y, max_z;

  bool is_in_box;
  // Keep track of the new indices when redoing the connections
  // for the mesh surfaces
  std::map<size_t, size_t> remapping, second_remapping;
  size_t original_size_all = all_vertices_.size();
  size_t original_size_active = active_vertices_xyz_->size();

  // Create temporary octree and active cloud and other temp structures
  PointCloudXYZ::Ptr temp_active_vertices(
      new PointCloudXYZ(*active_vertices_xyz_));
  PointCloud temp_all_vertices(all_vertices_);
  Octree temp_octree(*octree_);
  temp_octree.setInputCloud(temp_active_vertices);
  std::vector<size_t> temp_active_vertices_index(active_vertices_index_);
  std::vector<size_t> temp_new_indices;
  std::map<size_t, std::vector<size_t> > temp_adjacent_polygons(
      adjacent_polygons_);
  std::vector<pcl::Vertices> temp_polygons(polygons_);
  std::vector<pcl::Vertices> temp_new_triangles;

  //// First pass through with temporary variables
  for (size_t i = 0; i < input_vertices.size(); ++i) {
    const pcl::PointXYZRGBA p = input_vertices.points[i];
    const pcl::PointXYZ p_xyz(p.x, p.y, p.z);
    try {
      temp_octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
      is_in_box = (p_xyz.x >= min_x && p_xyz.x <= max_x) &&
                  (p_xyz.y >= min_y && p_xyz.y <= max_y) &&
                  (p_xyz.z >= min_z && p_xyz.z <= max_z);
      if (!is_in_box || !temp_octree.isVoxelOccupiedAtPoint(p_xyz)) {
        // New point. Update temp structures
        temp_active_vertices->push_back(p_xyz);
        // Add to (temp) octree
        temp_octree.addPointFromCloud(temp_active_vertices->size() - 1,
                                      nullptr);
        // Note that the other method to add to octree is addPointToCloud(point,
        // inputcloud) but this method causes segmentation faults under certain
        // conditions
        temp_all_vertices.push_back(p);
        // Track index for (first) remapping
        remapping[i] = temp_all_vertices.size() - 1;
        // Add (temp) index (temp active index to temp all index mapping)
        temp_active_vertices_index.push_back(temp_all_vertices.size() - 1);
        temp_new_indices.push_back(temp_all_vertices.size() - 1);
        temp_adjacent_polygons[temp_all_vertices.size() - 1] =
            std::vector<size_t>{};
      } else {
        // A nearby point exist, remap to nearby point
        float unused = 0.f;
        int result_idx;
        temp_octree.approxNearestSearch(p_xyz, result_idx, unused);
        // Add remapping index
        remapping[i] = temp_active_vertices_index[result_idx];
        // Push to new indices if does not already yet
        if (result_idx < original_size_active &&
            std::find(temp_new_indices.begin(),
                      temp_new_indices.end(),
                      temp_active_vertices_index[result_idx]) ==
                temp_new_indices.end()) {
          temp_new_indices.push_back(temp_active_vertices_index[result_idx]);
        }
      }
    } catch (...) {
      ROS_ERROR("OctreeCompression: Failed to insert mesh vertex. ");
    }
  }

  if (temp_new_indices.size() < 3) return;  // no surface after compression

  // Check polygons
  for (pcl::Vertices polygon : input_surfaces) {
    pcl::Vertices new_polygon;
    // Remap polygon while checking if polygon is new
    // by checking to see if any indices in new regime
    bool new_surface = false;
    for (size_t idx : polygon.vertices) {
      new_polygon.vertices.push_back(remapping[idx]);
      if (remapping[idx] >= original_size_all) new_surface = true;
    }

    // Check if polygon has actual three diferent vertices
    // To avoid degeneracy
    if (new_polygon.vertices.size() < 3 ||
        new_polygon.vertices[0] == new_polygon.vertices[1] ||
        new_polygon.vertices[1] == new_polygon.vertices[2] ||
        new_polygon.vertices[2] == new_polygon.vertices[0])
      continue;

    // Check if it is a new surface constructed from existing points
    if (!new_surface) {
      new_surface =
          !SurfaceExists(new_polygon, temp_adjacent_polygons, temp_polygons);
    }

    // If it is a new surface, add
    if (new_surface) {
      // Definitely a new surface
      temp_new_triangles.push_back(new_polygon);
      temp_polygons.push_back(new_polygon);
      // Update (temp) adjacent polygons
      for (size_t v : new_polygon.vertices) {
        temp_adjacent_polygons[v].push_back(temp_polygons.size() - 1);
      }
    }
  }
  if (temp_new_triangles.size() == 0) return;  // No new surfaces
  //// Second pass through to clean up
  // Second remapping to clean up the points that are not vertices (do not
  // belong to a face)
  for (auto idx : temp_new_indices) {
    // Check if point belongs to any surface of mesh
    if (temp_adjacent_polygons.at(idx).size() > 0) {
      if (idx >= original_size_all) {  // Check if a new point
        new_vertices->push_back(temp_all_vertices.points[idx]);
        // New point
        pcl::PointXYZRGBA new_p = temp_all_vertices.points[idx];
        // Insert to active vertices
        active_vertices_xyz_->push_back(
            pcl::PointXYZ(new_p.x, new_p.y, new_p.z));
        // Add to octree
        octree_->addPointFromCloud(active_vertices_xyz_->size() - 1, nullptr);
        // Insert to all vertices
        all_vertices_.push_back(new_p);
        // Create remapping (second remapping to not include the non-vertex
        // points)
        second_remapping[idx] = all_vertices_.size() - 1;
        // keep track of index (index in active -> index in all)
        active_vertices_index_.push_back(all_vertices_.size() - 1);
        new_indices->push_back(all_vertices_.size() - 1);
        // Add latest observed time
        vertices_latest_time_.push_back(stamp_in_sec);
        adjacent_polygons_[all_vertices_.size() - 1] = std::vector<size_t>{};
      } else {
        // Old point so no need to add to other structures
        second_remapping[idx] = idx;
        new_indices->push_back(idx);
        vertices_latest_time_[idx] = stamp_in_sec;
      }
    }
  }
  if (new_indices->size() == 0) return;  // no new indices so no new surfaces

  // Reindex the new surfaces using the second remapping
  for (auto t : temp_new_triangles) {
    pcl::Vertices reindexed_t;
    for (auto idx : t.vertices) {
      reindexed_t.vertices.push_back(second_remapping[idx]);
    }
    // Add to class and actual new triangles
    polygons_.push_back(reindexed_t);
    new_triangles->push_back(reindexed_t);
    // Update adjacent polygons
    for (size_t v : reindexed_t.vertices) {
      adjacent_polygons_[v].push_back(polygons_.size() - 1);
    }
  }
  return;
}

void OctreeCompression::pruneStoredMesh(const double& earliest_time_sec) {
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

      // Reset octree
      octree_.reset(new Octree(octree_resolution_));
      octree_->setInputCloud(active_vertices_xyz_);
      octree_->addPointsFromInputCloud();
    }
  } catch (...) {
    ROS_ERROR("OctreeCompression: Failed to prune active mesh. ");
  }
  return;
}

}  // namespace kimera_pgmo