/**
 * @file   example_from_ply.cpp
 * @brief  Demo for mesh distortion by importing from ply file
 * @author Yun Chang
 */
#include <iostream>

#include <ros/ros.h>

#include <pcl/PolygonMesh.h>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/DeformationGraph.h"

using mesher_mapper::DeformationGraph;
using mesher_mapper::ReadMeshFromPly;

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  std::cout << "etf " << std::endl;
  ros::init(argc, argv, "example_from_ply");
  std::cout << "argv1: " << argv[1] << std::endl;
  pcl::PolygonMeshPtr input_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(argv[1], input_mesh);
  std::cout << "mesh size orig: " << input_mesh->cloud.data.size() << std::endl;
  DeformationGraph graph(input_mesh);

  return 1;
}
