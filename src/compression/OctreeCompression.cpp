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
  vertices_.reset(new PointCloud);
  // Initialize octree
  octree_.reset(new Octree(octree_resolution_));
  octree_->setInputCloud(vertices_);
}

OctreeCompression::~OctreeCompression() {}

void OctreeCompression::compressAndIntegrate(
    const pcl::PolygonMesh& input,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices,
    std::vector<pcl::Vertices>* new_triangles,
    std::vector<size_t>* new_indices,
    const double& stamp_in_sec) {
  // Extract vertices from input mesh
  PointCloud::Ptr new_cloud(new PointCloud);
  pcl::fromPCLPointCloud2(input.cloud, *new_cloud);

  // Place vertices through octree for compression
  double min_x, min_y, min_z, max_x, max_y, max_z;

  bool is_in_box;
  // Keep track of the new indices when redoing the connections
  // for the mesh surfaces
  std::map<size_t, size_t> remapping;
  size_t original_size = vertices_->points.size();

  for (size_t i = 0; i < new_cloud->points.size(); ++i) {
    const pcl::PointXYZRGBA p = new_cloud->points[i];
    octree_->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
    is_in_box = (p.x >= min_x && p.x <= max_x) &&
                (p.y >= min_y && p.y <= max_y) &&
                (p.z >= min_z && p.z <= max_z);
    if (!is_in_box || !octree_->isVoxelOccupiedAtPoint(p)) {
      // New point
      new_vertices->push_back(p);
      octree_->addPointToCloud(p, vertices_);  // add to octree
      // Add index
      remapping[i] = vertices_->points.size() - 1;
      new_indices->push_back(vertices_->points.size() - 1);
      // Add latest observed time
      vertices_latest_time_.push_back(stamp_in_sec);
    } else {
      // A nearby point exist, remap to nearby point
      float unused = 0.f;
      int result_idx;
      octree_->approxNearestSearch(p, result_idx, unused);
      // Add remapping index
      remapping[i] = result_idx;
      new_indices->push_back(result_idx);
      if (result_idx < vertices_latest_time_.size())
        vertices_latest_time_.at(result_idx) = stamp_in_sec;
    }
  }

  // Insert polygons
  for (pcl::Vertices polygon : input.polygons) {
    pcl::Vertices new_polygon;
    // Remap polygon while checking if polygon is new
    // by checking to see if nay indices in new regime
    bool new_surface = false;
    for (size_t idx : polygon.vertices) {
      new_polygon.vertices.push_back(remapping[idx]);
      if (remapping[idx] < original_size) new_surface = true;
    }
    // Check if polygon has actual three diferent surfaces
    // To avoid degeneracy
    if (new_polygon.vertices[0] == new_polygon.vertices[1] ||
        new_polygon.vertices[1] == new_polygon.vertices[2] ||
        new_polygon.vertices[2] == new_polygon.vertices[0])
      continue;

    // If it is a new surface, add
    if (new_surface) {
      // Definitely a new surface
      polygons_.push_back(new_polygon);
      new_triangles->push_back(new_polygon);
    }
  }
}

void OctreeCompression::pruneStoredMesh(const double& earliest_time_sec) {
  // Entries in vertices_latest_time_ shoudl correspond to number of points
  if (vertices_latest_time_.size() != vertices_->points.size()) {
    ROS_ERROR(
        "Length of book-keeped vertex time does not match number of points. ");
  }
  // Discard all vertices last detected before this time
  PointCloud new_vertices;
  std::vector<double> new_vertices_time;
  std::vector<size_t> removed_indices;

  for (size_t i = 0; i < vertices_latest_time_.size(); i++) {
    if (vertices_latest_time_[i] > earliest_time_sec) {
      new_vertices_time.push_back(vertices_latest_time_[i]);
      new_vertices.push_back(vertices_->points[i]);
    } else {
      // Track the indices of the removed vertices
      removed_indices.push_back(i);
    }
  }

  // If discarded new points
  if (removed_indices.size() > 0) {
    // Set to not-removed vertices
    *vertices_ = new_vertices;
    vertices_latest_time_ = new_vertices_time;

    // Reset octree
    octree_.reset(new Octree(octree_resolution_));
    octree_->setInputCloud(vertices_);

    // Reset polygons
    std::vector<pcl::Vertices> new_polygons;
    for (pcl::Vertices polygon : polygons_) {
      bool remove = false;
      for (size_t idx : polygon.vertices) {
        // Check if polygon has removed vertices
        if (std::find(removed_indices.begin(), removed_indices.end(), idx) !=
            removed_indices.end())
          remove = true;
        break;
      }
      if (!remove) {
        new_polygons.push_back(polygon);
      }
    }
    polygons_ = new_polygons;
  }
}

}  // namespace kimera_pgmo
