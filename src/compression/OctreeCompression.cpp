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
  for (auto polygon : input_surfaces) {
    bool new_surface = false;
    pcl::Vertices new_polygon;
    std::vector<pcl::PointXYZRGBA> new_polygon_vertices;
    // Create temporary octree and cloud
    Octree temp_octree(octree_resolution_);
    PointCloud::Ptr temp_active_vertices(new PointCloud);
    *temp_active_vertices = *active_vertices_;
    temp_octree.setInputCloud(temp_active_vertices);
    temp_octree.addPointsFromInputCloud();
    size_t new_idx = temp_active_vertices->points.size();
    for (auto idx : polygon.vertices) {
      // Check / compress point using octree
      const pcl::PointXYZRGBA p = input_vertices.points[idx];
      new_polygon_vertices.push_back(p);
      try {
        double min_x, min_y, min_z, max_x, max_y, max_z;
        bool is_in_box;
        temp_octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
        is_in_box = (p.x >= min_x && p.x <= max_x) &&
                    (p.y >= min_y && p.y <= max_y) &&
                    (p.z >= min_z && p.z <= max_z);
        if (!is_in_box || !temp_octree.isVoxelOccupiedAtPoint(p)) {
          // New point
          new_surface = true;  // Must be a new surface if new point detected
          new_polygon.vertices.push_back(new_idx);
          new_idx++;
          temp_active_vertices->points.push_back(p);
          temp_octree.addPointFromCloud(temp_active_vertices->points.size() - 1,
                                        nullptr);
        } else {
          // A nearby point exists, remap to nearby point
          float unused = 0.f;
          int result_idx;
          temp_octree.approxNearestSearch(p, result_idx, unused);
          new_polygon.vertices.push_back(result_idx);
        }
      } catch (...) {
        ROS_ERROR("OctreeCompression: Failed to insert mesh vertex. ");
        return;
      }
    }

    // Check if polygon has actual three diferent vertices
    // To avoid degeneracy
    if (new_polygon.vertices.size() < 3 ||
        new_polygon.vertices[0] == new_polygon.vertices[1] ||
        new_polygon.vertices[1] == new_polygon.vertices[2] ||
        new_polygon.vertices[2] == new_polygon.vertices[0])
      continue;

    // Add to actual octree
    for (size_t i = 0; i < new_polygon_vertices.size(); i++) {
      if (new_polygon.vertices[i] >= active_vertices_->size()) {
        // new point
        const pcl::PointXYZRGBA p = new_polygon_vertices[i];
        new_vertices->push_back(p);
        adjacent_polygons_.push_back(std::vector<pcl::Vertices>());
        active_vertices_->points.push_back(p);
        octree_->addPointFromCloud(active_vertices_->points.size() - 1,
                                   nullptr);
        // Note that the other method to add to octree is
        // addPointToCloud(point, inputcloud) but this method causes
        // segmentation faults sometimes
        all_vertices_->push_back(p);
        new_polygon.vertices[i] = active_vertices_->points.size() - 1;
        active_vertices_index_.push_back(all_vertices_->points.size() - 1);
        new_indices->push_back(all_vertices_->points.size() - 1);
        vertices_latest_time_.push_back(stamp_in_sec);
      } else {
        // Not new point
        size_t result_idx = new_polygon.vertices[i];
        new_polygon.vertices[i] = active_vertices_index_[result_idx];
        if (std::find(new_indices->begin(),
                      new_indices->end(),
                      active_vertices_index_[result_idx]) ==
            new_indices->end()) {
          new_indices->push_back(active_vertices_index_[result_idx]);
        }
        vertices_latest_time_[result_idx] = stamp_in_sec;
      }
    }

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
  if (active_vertices_->points.size() == 0) return;  // nothing to prune
  // Entries in vertices_latest_time_ shoudl correspond to number of points
  if (vertices_latest_time_.size() != active_vertices_->points.size()) {
    ROS_ERROR(
        "Length of book-keeped vertex time does not match number of active "
        "points. ");
  }

  if (active_vertices_index_.size() != active_vertices_->points.size()) {
    ROS_ERROR(
        "Length of book-keeped vertex indices does not match number of "
        "active "
        "points. ");
  }

  try {
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
        active_vertices_->push_back(temp_active_vertices.points[i]);
        octree_->addPointFromCloud(active_vertices_->points.size() - 1,
                                   nullptr);
        vertices_latest_time_.push_back(temp_vertices_time[i]);
        active_vertices_index_.push_back(temp_vertices_index[i]);
      }
    }
  } catch (...) {
    ROS_ERROR("OctreeCompression: Failed to prune active mesh. ");
  }
}

}  // namespace kimera_pgmo
