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
  assert(nullptr != active_vertices_);
  assert(nullptr != all_vertices_);
  assert(nullptr != octree_);

  // Place vertices through octree for compression
  double min_x, min_y, min_z, max_x, max_y, max_z;

  bool is_in_box;
  // Keep track of the new indices when redoing the connections
  // for the mesh surfaces
  std::map<size_t, size_t> remapping, second_remapping;
  size_t original_size_all = all_vertices_->size();
  size_t original_size_active = active_vertices_->size();

  // Create temporary octree and active cloud and other temp structures
  PointCloud::Ptr temp_active_vertices(new PointCloud(*active_vertices_));
  PointCloud::Ptr temp_all_vertices(new PointCloud(*all_vertices_));
  Octree::Ptr temp_octree(new Octree(*octree_));
  temp_octree->setInputCloud(temp_active_vertices);
  std::vector<size_t> temp_active_vertices_index(active_vertices_index_);
  std::vector<size_t> temp_new_indices;
  std::vector<std::vector<pcl::Vertices> > temp_adjacent_polygons(
      adjacent_polygons_);
  std::vector<pcl::Vertices> temp_new_triangles;
  //// First pass through with temporary variables
  for (size_t i = 0; i < input_vertices.points.size(); ++i) {
    const pcl::PointXYZRGBA p = input_vertices.points[i];
    try {
      temp_octree->getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
      is_in_box = (p.x >= min_x && p.x <= max_x) &&
                  (p.y >= min_y && p.y <= max_y) &&
                  (p.z >= min_z && p.z <= max_z);
      if (!is_in_box || !temp_octree->isVoxelOccupiedAtPoint(p)) {
        // New point. Update temp structures
        temp_adjacent_polygons.push_back(std::vector<pcl::Vertices>());  // help
        temp_active_vertices->points.push_back(p);
        // Add to (temp) octree
        temp_octree->addPointFromCloud(temp_active_vertices->points.size() - 1,
                                       nullptr);
        // Note that the other method to add to octree is addPointToCloud(point,
        // inputcloud) but this method causes segmentation faults under certain
        // conditions
        temp_all_vertices->push_back(p);
        // Track index for (first) remapping
        remapping[i] = temp_all_vertices->points.size() - 1;
        // Add (temp) index (temp active index to temp all index mapping)
        temp_active_vertices_index.push_back(temp_all_vertices->points.size() -
                                             1);
        temp_new_indices.push_back(temp_all_vertices->points.size() - 1);
      } else {
        // A nearby point exist, remap to nearby point
        float unused = 0.f;
        int result_idx;
        temp_octree->approxNearestSearch(p, result_idx, unused);
        // Add remapping index
        remapping[i] = temp_active_vertices_index[result_idx];
        // Push to new indices if does not already yet
        if (result_idx < original_size_active &&
            std::find(temp_new_indices.begin(),
                      temp_new_indices.end(),
                      temp_active_vertices_index[result_idx]) ==
                temp_new_indices.end())
          temp_new_indices.push_back(temp_active_vertices_index[result_idx]);
      }
    } catch (...) {
      ROS_ERROR("OctreeCompression: Failed to insert mesh vertex. ");
    }
  }

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
      new_surface = !SurfaceExists(new_polygon, adjacent_polygons_);
    }

    // If it is a new surface, add
    if (new_surface) {
      // Definitely a new surface
      temp_new_triangles.push_back(new_polygon);
      // Update (temp) adjacent polygons
      for (size_t v : new_polygon.vertices) {
        temp_adjacent_polygons[v].push_back(new_polygon);
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
        new_vertices->push_back(temp_all_vertices->points[idx]);
        adjacent_polygons_.push_back(std::vector<pcl::Vertices>());
        active_vertices_->points.push_back(temp_all_vertices->points[idx]);
        // Add to octree
        octree_->addPointFromCloud(active_vertices_->points.size() - 1,
                                   nullptr);
        all_vertices_->push_back(temp_all_vertices->points[idx]);
        // Create remapping (second remapping to not include the non-vertex
        // points)
        second_remapping[idx] = all_vertices_->points.size() - 1;
        // keep track of index (index in active -> index in all)
        active_vertices_index_.push_back(all_vertices_->points.size() - 1);
        new_indices->push_back(all_vertices_->points.size() - 1);
        // Add latest observed time
        vertices_latest_time_.push_back(stamp_in_sec);
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
      adjacent_polygons_[v].push_back(reindexed_t);
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