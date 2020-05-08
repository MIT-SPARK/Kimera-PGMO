/**
 * @file   AlliezDesbrunCompression.cpp
 * @brief  Simplify and reconstruct meshes Alliez Desbrun 2001
 * @author Yun Chang
 */
#include "mesher_mapper/AlliezDesbrunCompression.h"

namespace mesher_mapper {

AlliezDesbrunCompression::AlliezDesbrunCompression() {}
AlliezDesbrunCompression::~AlliezDesbrunCompression() {}

bool AlliezDesbrunCompression::process() {}

bool AlliezDesbrunCompression::setInputMesh(pcl::PolygonMeshPtr input_mesh) {
  original_mesh_ = *input_mesh;
  // Generate mesh graph
  mesh_graph_.createFromPclMesh(*input_mesh);
  // Set vertices 
  for ()

}

void AlliezDesbrunCompression::DecimatingConquest(
    pcl::PolygonMeshPtr output_mesh,
    GraphPtr ouput_graph) {
	// Define and populate vertex status 
	std::map<Vertex, VertexStatus> vertex_status; 


}

}  // namespace mesher_mapper