/**
 * @file   OctreeCompression.h
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#pragma once

#include <pcl/octree/octree_search.h>

#include "mesher_mapper/MeshCompression.h"
#include "mesher_mapper/Polygon.h"

namespace mesher_mapper {

class OctreeCompression : public MeshCompression {
 public:
  typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;

  OctreeCompression();
  ~OctreeCompression();

  bool setInputMesh(pcl::PolygonMeshPtr input_mesh);
  bool process();

  inline void setResolution(double resolution) { resolution_ = resolution; }

 private:
  bool reset(const pcl::PolygonMesh& mesh);
  bool ConstructMeshFromCloud(
      const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
      double search_radius,
      pcl::PolygonMeshPtr mesh);

  Octree::Ptr octree_;
  PointCloud::Ptr octree_points_; 

  double resolution_;
};
}  // namespace mesher_mapper