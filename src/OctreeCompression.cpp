/**
 * @file   OctreeCompression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <algorithm>
#include <cmath>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include "mesher_mapper/OctreeCompression.h"

namespace mesher_mapper {

OctreeCompression::OctreeCompression() {}
OctreeCompression::~OctreeCompression() {}

bool OctreeCompression::setInputMesh(pcl::PolygonMeshPtr input_mesh) {
  original_mesh_ = *input_mesh;

  reset(*input_mesh);
}
bool OctreeCompression::process() {
  // Build octree
}
bool OctreeCompression::reset(const pcl::PolygonMesh& mesh) {
  // clear everything
  vertex_positions_.clear();
  // populate vertices data
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);
  // Populate vertex positions and add to first block of octree
  octree_.reset(new Octree(0.01));
  octree_->setInputCloud(cloud);
}

// void OctreeCompression::splitIntoEight(
//     const OctreeBlockPtr& parent,
//     std::shared_ptr<std::map<Vertex, size_t>> vertex_to_block_id) const {
//   // Block ID: from 0 to 7
//   // (x+,y+,z+), (x-,y+,z+), (x+,y-,z+), (x-,y-,z+), (x+,y+,z-), (x-,y+,z-),
//   // (x+,y-,z-), (x-,y-,z-)

//   // First get x y z median
//   std::vector<std::vector<double>> values;
//   Pointxyz median;
//   for (Vertex v : *parent) {
//     for (size_t j = 0; j < 3; j++) {
//       values[j].push_back(vertex_positions_.at(v)[j]);
//     }
//   }
//   for (size_t j = 0; j < 3; j++) {
//     std::sort(values[j].begin(), values[j].end());
//     size_t n = values[j].size();
//     median[j] =
//         (values[j][std::floor(n / 2) - 1] + values[j][std::ceil(n / 2) - 1])
//         / 2;
//   }

//   // Place into one of eight containers
//   for (Vertex v : *parent) {
//     std::vector<int> signs;
//     for (size_t i = 0; i < 3; i++) {
//       if (vertex_positions_.at(v)[i] > median[i]) {
//         signs.push_back(0);
//       } else {
//         signs.push_back(1);
//       }
//     }
//     // find corresponding index (container) and place
//     size_t idx = signs[0] + 2 * signs[1] + 4 * signs[2];
//     vertex_to_block_id->at(v) = idx;
//   }
// }
}  // namespace mesher_mapper