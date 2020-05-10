/**
 * @file   alliez_desbrun.cpp
 * @brief  Demo for mesh compression
 * @author Yun Chang
 */
#include <iostream>

#include <ros/ros.h>

#include <pcl/PolygonMesh.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "mesher_mapper/AlliezDesbrunCompression.h"
#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/CommonStructs.h"
#include "mesher_mapper/DeformationGraph.h"
#include "mesher_mapper/Polygon.h"

using mesher_mapper::AlliezDesbrunCompression;
using mesher_mapper::DeformationGraph;
using mesher_mapper::Polygon;
using mesher_mapper::ReadMeshFromPly;
using mesher_mapper::Vertex;
using mesher_mapper::Vertices;

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "example_from_ply");
  pcl::PolygonMeshPtr input_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(argv[1], input_mesh);
  // visualize
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(0, 0, 0);
  viewer->addPolygonMesh(*input_mesh, "input mesh");
  viewer->spin();
  AlliezDesbrunCompression ad_compress;
  ad_compress.setInputMesh(input_mesh);
  ad_compress.process();
  pcl::PolygonMeshPtr simplified_mesh(new pcl::PolygonMesh());
  ad_compress.getBaseMesh(simplified_mesh);
  std::cout << "number of triangles of input mesh: "
            << input_mesh->polygons.size() << std::endl;
  std::cout << "number of triangles of simplifed mesh: "
            << simplified_mesh->polygons.size() << std::endl;
  viewer->addPolygonMesh(*simplified_mesh, "simplified mesh");
  viewer->removePolygonMesh("input mesh");

  viewer->spin();

  return 1;
}
