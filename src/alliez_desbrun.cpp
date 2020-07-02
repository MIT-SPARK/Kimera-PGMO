/**
 * @file   alliez_desbrun.cpp
 * @brief  Demo for mesh compression
 * @author Yun Chang
 */
#include <iostream>

#include <ros/ros.h>

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "kimera_pgmo/AlliezDesbrunCompression.h"
#include "kimera_pgmo/CommonFunctions.h"
#include "kimera_pgmo/CommonStructs.h"
#include "kimera_pgmo/Polygon.h"

using kimera_pgmo::AlliezDesbrunCompression;
using kimera_pgmo::Polygon;
using kimera_pgmo::ReadMeshFromPly;
using kimera_pgmo::Vertex;
using kimera_pgmo::Vertices;

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "example_from_ply");
  pcl::PolygonMeshPtr input_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(argv[1], input_mesh);
  // visualize
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("3D Viewer"));
  viewer->setBackgroundColor(255, 255, 255);
  // viewer->addPolygonMesh(*input_mesh, "input mesh");
  // viewer->spin();
  AlliezDesbrunCompression ad_compress;
  ad_compress.setInputMesh(input_mesh);
  ad_compress.process();
  pcl::PolygonMeshPtr simplified_mesh(new pcl::PolygonMesh());
  ad_compress.getBaseMesh(simplified_mesh);
  while (simplified_mesh->polygons.size() > std::atoi(argv[3])) {
    ad_compress = AlliezDesbrunCompression();
    ad_compress.setInputMesh(simplified_mesh);
    ad_compress.process();
    ad_compress.getBaseMesh(simplified_mesh);
  }
  // pcl::io::savePLYFile(argv[2], *simplified_mesh);

  std::cout << "number of triangles of input mesh: "
            << input_mesh->polygons.size() << std::endl;
  std::cout << "number of triangles of simplifed mesh: "
            << simplified_mesh->polygons.size() << std::endl;
  viewer->addPolygonMesh(*simplified_mesh, "simplified mesh");
  // viewer->removePolygonMesh("input mesh");

  viewer->spin();

  return 1;
}
