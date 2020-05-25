/**
 * @file   CommonStructs.cpp
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#include <algorithm>
#include <numeric>

#include <pcl_conversions/pcl_conversions.h>

#include "mesher_mapper/CommonStructs.h"

namespace mesher_mapper {

//// Graph Class
std::vector<Edge> Graph::getEdges() const {
  std::vector<Edge> edges;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    Vertex v1 = it->first;
    for (Vertex v2 : it->second) {
      Edge e(v1, v2);
      edges.push_back(e);
    }
  }
  return edges;
}

void Graph::addEdgeAndVertices(const Edge& e) {
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
  addEdge(e);
}

void Graph::addEdge(const Edge& e) {
  // Push edge
  Edges::iterator iter = edges_.find(e.first);
  if (iter == edges_.end()) {
    edges_[e.first] = Vertices{e.second};
  } else {
    std::vector<Vertex>::iterator iter2;
    iter2 = std::find(edges_[e.first].begin(), edges_[e.first].end(), e.second);
    if (iter2 == edges_[e.first].end()) edges_[e.first].push_back(e.second);
  }
}

bool Graph::createFromPclMesh(const pcl::PolygonMesh& mesh) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  size_t n = cloud.points.size();
  vertices_ = std::vector<Vertex>(n);
  std::iota(std::begin(vertices_), std::end(vertices_), 0);
  for (Vertex v : vertices_) {
    edges_[v] = Vertices();
  }
  for (pcl::Vertices polygon : mesh.polygons) {
    for (size_t i = 0; i < polygon.vertices.size(); i++) {
      size_t i_next = (i + 1) % polygon.vertices.size();
      Edge e(polygon.vertices[i], polygon.vertices[i_next]);
      addEdge(e);
    }
  }
  return true;
}

bool Graph::createFromPclMeshBidirection(const pcl::PolygonMesh& mesh) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  size_t n = cloud.points.size();
  vertices_ = std::vector<Vertex>(n);
  std::iota(std::begin(vertices_), std::end(vertices_), 0);
  for (Vertex v : vertices_) {
    edges_[v] = Vertices();
  }
  for (pcl::Vertices polygon : mesh.polygons) {
    for (size_t i = 0; i < polygon.vertices.size(); i++) {
      size_t i_next = (i + 1) % polygon.vertices.size();
      Edge e1(polygon.vertices[i], polygon.vertices[i_next]);
      addEdge(e1);
      Edge e2(polygon.vertices[i_next], polygon.vertices[i]);
      addEdge(e2);
    }
  }
  return true;
}

bool Graph::combineGraph(const Graph& new_graph) {
  std::vector<Edge> new_edges = new_graph.getEdges();

  for (Edge e : new_edges) {
    addEdgeAndVertices(e);
  }

  return true;
}

void Graph::print(std::string header) const {
  // std::cout << header << "\n";
  // std::cout << "vertices: \n";
  // for (Vertex v : vertices_) {
  //   std::cout << v << " ";
  // }
  // std::cout << std::endl;
  std::cout << "edges: \n";
  Edges::const_iterator iter;
  for (iter = edges_.begin(); iter != edges_.end(); iter++) {
    for (Vertex v : iter->second) {
      std::cout << iter->first << "-->" << v << " ";
    }
  }
  std::cout << std::endl;
}

}  // namespace mesher_mapper
