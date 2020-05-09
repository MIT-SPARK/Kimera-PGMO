/**
 * @file   example_from_ply.cpp
 * @brief  Demo for mesh distortion by importing from ply file
 * @author Yun Chang
 */
#include <iostream>

#include <ros/ros.h>

#include <pcl/PolygonMesh.h>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/CommonStructs.h"
#include "mesher_mapper/DeformationGraph.h"
#include "mesher_mapper/Polygon.h"

using mesher_mapper::DeformationGraph;
using mesher_mapper::Polygon;
using mesher_mapper::ReadMeshFromPly;
using mesher_mapper::Vertex;
using mesher_mapper::Vertices;

int main(int argc, char* argv[]) {
  // // Initialize ROS node.
  // std::cout << "etf " << std::endl;
  // ros::init(argc, argv, "example_from_ply");
  // std::cout << "argv1: " << argv[1] << std::endl;
  // pcl::PolygonMeshPtr input_mesh(new pcl::PolygonMesh());
  // ReadMeshFromPly(argv[1], input_mesh);
  // std::cout << "mesh size orig: " << input_mesh->cloud.data.size() <<
  // std::endl; DeformationGraph graph(input_mesh);
  Vertices p_v{1, 2, 3, 4};
  Vertices p_v2{3, 5, 6, 4};
  Polygon<Vertex> p(p_v);
  Polygon<Vertex> p2(p_v2);
  p.print("p1");
  p2.print("p2");

  Polygon<Vertex> p12 = p.combine(p2);
  p12.print("p1 + p2");

  std::vector<size_t> tri_ind{0, 2, 4};
  std::shared_ptr<std::vector<Polygon<Vertex>>> tris =
      std::make_shared<std::vector<Polygon<Vertex>>>();
  p12.triangulateWithRefidx(tri_ind, tris);
  for (auto pt : *tris) {
  	pt.print("triang");
  }
  return 1;
}
