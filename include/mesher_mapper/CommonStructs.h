/**
 * @file   CommonStructs.h
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#pragma once

#include <map>
#include <string>
#include <utility>
#include <vector>

#include <pcl/PolygonMesh.h>

namespace mesher_mapper {

typedef unsigned int Vertex;
typedef std::vector<Vertex> Vertices;
typedef std::map<Vertex, Vertices> Edges;
typedef std::pair<Vertex, Vertex> Edge;

class Graph {
 public:
  inline Vertices getValence(const Vertex& v) const { return edges_.at(v); }
  inline Vertices getVertices() const { return vertices_; }

  void addEdge(const Edge& e);
  void addEdgeAndVertices(const Edge& e);

  bool createFromPclMesh(const pcl::PolygonMesh& mesh);
  void print(std::string header) const;

 private:
  Vertices vertices_;
  Edges edges_;
};

typedef std::shared_ptr<Graph> GraphPtr;

}  // namespace mesher_mapper
