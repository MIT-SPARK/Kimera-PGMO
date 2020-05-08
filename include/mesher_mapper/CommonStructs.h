/**
 * @file   CommonStructs.h
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#pragma once

#include <map>
#include <utility>
#include <vector>

#include <pcl/PolygonMesh.h>

namespace mesher_mapper {

typedef size_t Vertex;
typedef std::vector<Vertex> Vertices;
typedef std::map<Vertex, Vertices> Edges;
typedef std::pair<Vertex, Vertex> Edge;

class Graph {
 public:
  inline Vertices getValence(const Vertex& v) const { return edges_.at(v); }

  void addBidirectionEdge(const Edge& e);
  void addBidirectionEdgeAndVertices(const Edge& e);

  bool createFromPclMesh(const pcl::PolygonMesh& mesh);

 private:
  std::vector<Vertex> vertices_;
  Edges edges_;
};

typedef std::shared_ptr<Graph> GraphPtr;

}  // namespace mesher_mapper
