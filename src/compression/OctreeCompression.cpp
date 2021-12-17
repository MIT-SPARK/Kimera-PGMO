/**
 * @file   OctreeCompression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <algorithm>
#include <iterator>
#include <utility>

#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {

OctreeCompression::OctreeCompression(double resolution)
    : MeshCompression(resolution) {
  active_vertices_xyz_.reset(new PointCloudXYZ);
  // Initialize octree
  octree_.reset(new Octree(resolution));
  octree_->setInputCloud(active_vertices_xyz_);
  temp_octree_.reset(new Octree(resolution));
}

OctreeCompression::~OctreeCompression() {}

void OctreeCompression::reInitializeStructure(
    PointCloudXYZ::Ptr active_vertices) {
  // Reset octree
  octree_.reset(new Octree(resolution_));
  octree_->setInputCloud(active_vertices);
  octree_->addPointsFromInputCloud();
}

bool OctreeCompression::checkIfVertexUnique(const pcl::PointXYZ& v,
                                            int* matched_ind) const {
  if (!InOctreeBoundingBox<pcl::PointXYZ>(*octree_, v) ||
      !octree_->isVoxelOccupiedAtPoint(v)) {
    return true;
  } else {
    float unused = 0.f;
    octree_->approxNearestSearch(v, *matched_ind, unused);
    return false;
  }
}

void OctreeCompression::updateStructure(PointCloudXYZ::Ptr vertices) {
  octree_->addPointFromCloud(vertices->size() - 1, nullptr);
}

bool OctreeCompression::checkIfVertexTempUnique(const pcl::PointXYZ& v,
                                                int* matched_ind) const {
  if (!InOctreeBoundingBox<pcl::PointXYZ>(*temp_octree_, v) ||
      !temp_octree_->isVoxelOccupiedAtPoint(v)) {
    return true;
  } else {
    float unused = 0.f;
    temp_octree_->approxNearestSearch(v, *matched_ind, unused);
    return false;
  }
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