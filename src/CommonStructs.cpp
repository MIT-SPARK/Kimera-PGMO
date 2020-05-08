/**
 * @file   CommonStructs.cpp
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#include <algorithm>
#include <numeric>

#include "mesher_mapper/CommonStructs.h"

namespace mesher_mapper {

//// Graph Class
void Graph::addBidirectionEdgeAndVertices(const Edge& e) {
  Vertex v1 = e.first;
  Vertex v2 = e.second;
  // Search if vertices exist, otherwise push
  bool exist_v1 = false;
  bool exist_v2 = false;
  for (Vertex v : vertices_) {
    if (v == v1) exist_v1 = true;
    if (v == v2) exist_v2 = true;
    if (exist_v1 and exist_v2) break;
  }
  if (!exist_v1) {
    vertices_.push_back(v1);
  }
  if (!exist_v2) {
    vertices_.push_back(v2);
  }
  addBidirectionEdge(e);
}

void Graph::addBidirectionEdge(const Edge& e) {
  // Push edge
  Edges::iterator iter;
  bool exist_v1 = false;
  bool exist_v2 = false;
  for (iter = edges_.begin(); iter != edges_.end(); iter++) {
    if (iter->first == e.first) exist_v1 = true;
    if (iter->first == e.second) exist_v2 = true;
    if (exist_v1 and exist_v2) break;
  }
  if (!exist_v1) {
    edges_.at(e.first) = Vertices{e.second};
  } else {
    edges_.at(e.first).push_back(e.second);
  }
  if (!exist_v2) {
    edges_.at(e.second) = Vertices{e.first};
  } else {
    edges_.at(e.second).push_back(e.first);
  }
}

bool Graph::createFromPclMesh(const pcl::PolygonMesh& mesh) {
  size_t n = mesh.cloud.data.size();
  vertices_ = std::vector<size_t>(n);
  std::iota(std::begin(vertices_), std::end(vertices_), 0);
  for (pcl::Vertices polygon : mesh.polygons) {
    for (size_t i; i < polygon.vertices.size(); i++) {
      size_t i_next = (i + 1) % polygon.vertices.size();
      Edge e(polygon.vertices[i], polygon.vertices[i_next]);
      addBidirectionEdge(e);
    }
  }
}

bool Polygon::equal(const Polygon& p) const {
	vertices_equl = false; 
	p_starting_idx = 0; 
	while p_starting_idx < p.num_vertices(); 
	// If vertex labels populated check that also 
	vertex_labels_equal = false; 
	if (vertex)
	//
}
  Polygon combine(const Polygon& p) const;
  void triangulate(std::shared_ptr<std::vector<Polygon>> trianlges_ptr);
  void triangulateWithRefidx(
      const Vertices& ref_indices,
      std::shared_ptr<std::vector<Polygon>> triangles_ptr);

}  // namespace mesher_mapper
