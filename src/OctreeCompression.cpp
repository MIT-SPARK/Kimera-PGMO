/**
 * @file   OctreeCompression.cpp
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <pcl/PCLPointCloud2.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
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
  pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(original_mesh_.cloud, *input_cloud);
  std::cout << "original cloud size: " << input_cloud->size() << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_cloud(
      new pcl::PointCloud<pcl::PointXYZ>());

  pcl::UniformSampling<pcl::PointXYZ> uniform_sampling;
  uniform_sampling.setInputCloud(input_cloud);
  uniform_sampling.setRadiusSearch(0.001);
  uniform_sampling.filter(*downsampled_cloud);
  std::cout << "downsampled_cloud size: " << downsampled_cloud->size()
            << std::endl;

  // Create mesh
  pcl::PolygonMeshPtr new_mesh;
  ConstructMeshFromCloud(downsampled_cloud, 0.1, new_mesh);
  base_mesh_ = *new_mesh;
  return true;
}

bool OctreeCompression::reset(const pcl::PolygonMesh& mesh) {
  // clear everything
  vertex_positions_.clear();
  // populate vertices data
}

bool OctreeCompression::ConstructMeshFromCloud(
    const pcl::PointCloud<pcl::PointXYZ>::ConstPtr& cloud,
    double search_radius,
    pcl::PolygonMeshPtr mesh) {
  ROS_INFO("Constructing mesh from downsampled cloud");
  // Estimate normals
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  pcl::PointCloud<pcl::Normal>::Ptr normals(new pcl::PointCloud<pcl::Normal>);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(
      new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(cloud);
  n.setInputCloud(cloud);
  n.setSearchMethod(tree);
  n.setKSearch(20);
  n.compute(*normals);

  // Concatenate the XYZ and normal fields*
  pcl::PointCloud<pcl::PointNormal>::Ptr cloud_with_normals(
      new pcl::PointCloud<pcl::PointNormal>);
  pcl::concatenateFields(*cloud, *normals, *cloud_with_normals);
  std::cout << "computed normals. Size: " << cloud_with_normals->size()
            << std::endl;

  // Create search tree*
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree2(
      new pcl::search::KdTree<pcl::PointNormal>);
  std::cout << "test" << std::endl;
  for (size_t i = 0; i < cloud_with_normals->points.size(); i++) {
    std::cout << cloud_with_normals->points[i];
  }
  std::cout << std::endl;
  tree2->setInputCloud(cloud_with_normals);
  std::cout << "Initialized kdtree" << std::endl;

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  pcl::PolygonMesh triangles;
  std::cout << "Initialized greedy projection triangulation" << std::endl;
  // Set the maximum distance between connected points (maximum edge length)
  gp3.setSearchRadius(search_radius);

  // Set typical values for the parameters
  gp3.setMu(3);
  gp3.setMaximumNearestNeighbors(100);
  // gp3.setMaximumSurfaceAngle(M_PI / 4);  // 45 degrees
  // gp3.setMinimumAngle(M_PI / 18);        // 10 degrees
  // gp3.setMaximumAngle(2 * M_PI / 3);     // 120 degrees
  // gp3.setNormalConsistency(false);

  // Get result
  gp3.setInputCloud(cloud_with_normals);
  gp3.setSearchMethod(tree2);
  gp3.reconstruct(triangles);

  *mesh = triangles;
  return true;
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