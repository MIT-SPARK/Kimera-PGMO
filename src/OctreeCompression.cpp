/**
 * @file   OctreeCompression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include "mesher_mapper/OctreeCompression.h"

namespace mesher_mapper {

OctreeCompression::OctreeCompression() : resolution_(0.01) {
  octree_points_.reset(new PointCloud);
}
OctreeCompression::~OctreeCompression() {}

bool OctreeCompression::setInputMesh(pcl::PolygonMeshPtr input_mesh) {
  original_mesh_ = *input_mesh;
  pcl::fromPCLPointCloud2(input_mesh->cloud, *octree_points_);

  // Reset and populate octree
  octree_.reset(new Octree(resolution_));
  // octree_->setInputCloud(octree_points_);

  reset(*input_mesh);
}
bool OctreeCompression::process() {
  PointCloud new_pts;
  new_pts.points = octree_points_->points;
  octree_points_.reset(new PointCloud);
  for (size_t i = 0; i < new_pts.points.size(); i++) {
    pcl::PointXYZ p = new_pts.points[i];
    // std::vector<int> neighbors;
    std::cout << p.x << "," << p.y << "," << p.z << std::endl;
    octree_->addPointToCloud(p, octree_points_);
    // std::cout << "neighbors of " << i << ": ";
    double minx, miny, minz, maxx, maxy, maxz;
    octree_->getBoundingBox(minx, miny, minz, maxx, maxy, maxz);
    std::cout << minx << miny << minz << maxx << maxy << maxz << std::endl;
    // bool isInBox = (p.x >= minx && p.y <= maxx) &&
    //                (p.y >= miny && p.y <= maxy) && (p.z >= minz && p.z <= maxz);

    // if (isInBox) {
    //   octree_->voxelSearch(i, neighbors);
    //   for (int idx : neighbors) {
    //     std::cout << idx << " ";
    //   }
    // } else {
    //   // octree_->addPointToCloud(p, octree_points_);
    // }
    // std::cout << std::endl;
  }
}

bool OctreeCompression::reset(const pcl::PolygonMesh& mesh) {}
}  // namespace mesher_mapper