/**
 * @file   octree_compression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include "kimera_pgmo/compression/octree_compression.h"

#include <algorithm>
#include <iterator>
#include <utility>

#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

// TODO(nathan) this still fails in weird places if we're on a voxel border
template <typename T>
bool inOctree(pcl::octree::OctreePointCloudSearch<T>& octree,
              const pcl::PointXYZ& v,
              int* matched_ind) {
  if (!octree.getLeafCount()) {
    // octree is empty, so bottom out
    return false;
  }

  if (!InOctreeBoundingBox<T>(octree, v)) {
    return false;
  }

  const auto resolution = octree.getResolution();
  const auto threshold = resolution * resolution;
  float sqr_distance = threshold + 1.0f;
  octree.approxNearestSearch(v, *matched_ind, sqr_distance);
  if (sqr_distance >= threshold) {
    *matched_ind = -1;
    return false;
  }

  return true;
}

OctreeCompression::OctreeCompression(double resolution) : MeshCompression(resolution) {
  active_vertices_xyz_.reset(new PointCloudXYZ);
  // Initialize octree
  octree_.reset(new Octree(resolution));
  octree_->setInputCloud(active_vertices_xyz_);
  temp_octree_.reset(new Octree(resolution));
}

OctreeCompression::~OctreeCompression() {}

void OctreeCompression::reInitializeStructure(PointCloudXYZ::Ptr active_vertices) {
  // Reset octree
  octree_.reset(new Octree(resolution_));
  octree_->setInputCloud(active_vertices);
  octree_->addPointsFromInputCloud();
}

bool OctreeCompression::checkIfVertexUnique(const pcl::PointXYZ& v,
                                            int* matched_ind) const {
  return !inOctree(*octree_, v, matched_ind);
}

void OctreeCompression::updateStructure(PointCloudXYZ::Ptr vertices) {
  octree_->addPointFromCloud(vertices->size() - 1, nullptr);
}

bool OctreeCompression::checkIfVertexTempUnique(const pcl::PointXYZ& v,
                                                int* matched_ind) const {
  return !inOctree(*temp_octree_, v, matched_ind);
}

void OctreeCompression::initializeTempStructure(PointCloudXYZ::Ptr vertices) {
  temp_octree_.reset(new Octree(resolution_));
  temp_octree_->setInputCloud(vertices);
  temp_octree_->addPointsFromInputCloud();
}

void OctreeCompression::updateTempStructure(PointCloudXYZ::Ptr vertices) {
  temp_octree_->addPointFromCloud(vertices->size() - 1, nullptr);
}

}  // namespace kimera_pgmo
