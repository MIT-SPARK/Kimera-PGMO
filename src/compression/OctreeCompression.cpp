/**
 * @file   OctreeCompression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <algorithm>
#include <map>

#include <ros/ros.h>

#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {

OctreeCompression::OctreeCompression(double resolution)
    : octree_resolution_(resolution) {
  active_vertices_.reset(new PointCloud);
  all_vertices_.reset(new PointCloud);
  // Initialize octree
  octree_.reset(new Octree(octree_resolution_));
  octree_->setInputCloud(active_vertices_);
}

OctreeCompression::~OctreeCompression() {}

void OctreeCompression::compressAndIntegrate(
    const pcl::PolygonMesh& input,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
    std::vector<pcl::Vertices>* new_triangles,
    std::vector<size_t>* new_indices,
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
}

void OctreeCompression::compressAndIntegrate(
    const pcl::PointCloud<pcl::PointXYZRGBA>& input_vertices,
    const std::vector<pcl::Vertices>& input_surfaces,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
    std::vector<pcl::Vertices>* new_triangles,
    std::vector<size_t>* new_indices,
    const double& stamp_in_sec) {
  // Place vertices through octree for compression
  double min_x, min_y, min_z, max_x, max_y, max_z;

  bool is_in_box;
  // Keep track of the new indices when redoing the connections
  // for the mesh surfaces
  std::map<size_t, size_t> remapping;
  size_t original_size = all_vertices_->points.size();

  for (size_t i = 0; i < input_vertices.points.size(); ++i) {
    const pcl::PointXYZRGBA p = input_vertices.points[i];
    octree_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    is_in_box = (p.x >= min_x && p.x <= max_x) &&
                (p.y >= min_y && p.y <= max_y) &&
                (p.z >= min_z && p.z <= max_z);
    if (!is_in_box || !octree_->isVoxelOccupiedAtPoint(p)) {
      // New point
      new_vertices->push_back(p);
      adjacent_polygons_.push_back(std::vector<pcl::Vertices>());
      octree_->addPointToCloud(p, active_vertices_);  // add to octree
      all_vertices_->push_back(p);
      // Add index
      remapping[i] = all_vertices_->points.size() - 1;
      // keep track of index
      active_vertices_index_.push_back(all_vertices_->points.size() - 1);
      new_indices->push_back(all_vertices_->points.size() - 1);
      // Add latest observed time
      vertices_latest_time_.push_back(stamp_in_sec);
    } else {
      // A nearby point exist, remap to nearby point
      float unused = 0.f;
      int result_idx;
      octree_->approxNearestSearch(p, result_idx, unused);
      // Add remapping index
      remapping[i] = active_vertices_index_[result_idx];
      // Push to new indices if does not already yet
      if (std::find(new_indices->begin(),
                    new_indices->end(),
                    active_vertices_index_[result_idx]) == new_indices->end())
        new_indices->push_back(active_vertices_index_[result_idx]);
      if (result_idx < vertices_latest_time_.size())
        vertices_latest_time_.at(result_idx) = stamp_in_sec;
    }
  }

  // Insert polygons
  for (pcl::Vertices polygon : input_surfaces) {
    pcl::Vertices new_polygon;
    // Remap polygon while checking if polygon is new
    // by checking to see if nay indices in new regime
    bool new_surface = false;
    for (size_t idx : polygon.vertices) {
      new_polygon.vertices.push_back(remapping[idx]);
      if (remapping[idx] >= original_size) new_surface = true;
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
      new_surface = !SurfaceExists(new_polygon, adjacent_polygons_);
    }

    // If it is a new surface, add
    if (new_surface) {
      // Definitely a new surface
      polygons_.push_back(new_polygon);
      new_triangles->push_back(new_polygon);
      // Update adjacent polygons
      for (size_t v : new_polygon.vertices) {
        adjacent_polygons_[v].push_back(new_polygon);
      }
    }
  }
}

void OctreeCompression::pruneStoredMesh(const double& earliest_time_sec) {
  // Entries in vertices_latest_time_ shoudl correspond to number of points
  if (vertices_latest_time_.size() != active_vertices_->points.size()) {
    ROS_ERROR(
        "Length of book-keeped vertex time does not match number of active "
        "points. ");
  }

  if (active_vertices_index_.size() != active_vertices_->points.size()) {
    ROS_ERROR(
        "Length of book-keeped vertex indices does not match number of active "
        "points. ");
  }
  // Discard all vertices last detected before this time
  PointCloud temp_active_vertices = *active_vertices_;
  std::vector<double> temp_vertices_time = vertices_latest_time_;
  std::vector<size_t> temp_vertices_index = active_vertices_index_;

  active_vertices_->clear();
  vertices_latest_time_.clear();
  active_vertices_index_.clear();

  // Reset octree
  octree_.reset(new Octree(octree_resolution_));
  octree_->setInputCloud(active_vertices_);

  for (size_t i = 0; i < temp_vertices_time.size(); i++) {
    if (temp_vertices_time[i] > earliest_time_sec) {
      octree_->addPointToCloud(temp_active_vertices.points[i],
                               active_vertices_);  // add to octree
      vertices_latest_time_.push_back(temp_vertices_time[i]);
      active_vertices_index_.push_back(temp_vertices_index[i]);
    }
  }
}

}  // namespace kimera_pgmo
