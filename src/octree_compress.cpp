/**
 * @file   octree_compress.cpp
 * @brief  Demo for mesh compression
 * @author Yun Chang
 */
#include <iostream>

#include <ros/ros.h>

#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "mesher_mapper/OctreeCompression.h"
#include "mesher_mapper/CommonFunctions.h"
// #include "mesher_mapper/CommonStructs.h"
// #include "mesher_mapper/DeformationGraph.h"
// #include "mesher_mapper/Polygon.h"

using mesher_mapper::OctreeCompression;
// using mesher_mapper::DeformationGraph;
// using mesher_mapper::Polygon;
using mesher_mapper::ReadMeshFromPly;
// using mesher_mapper::Vertex;
// using mesher_mapper::Vertices;

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "octree_compress");
  pcl::PolygonMeshPtr input_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(argv[1], input_mesh);
  // // visualize
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  // viewer->setBackgroundColor(0, 0, 0);
  // // viewer->addPolygonMesh(*input_mesh, "input mesh");
  // // viewer->spin();
  OctreeCompression ot_compress;
  ot_compress.setInputMesh(input_mesh);
  ot_compress.setResolution(std::atof(argv[2]));
  ot_compress.process();
  // viewer->addPolygonMesh(*simplified_mesh, "simplified mesh");

  // viewer->spin();

  return 1;
}
