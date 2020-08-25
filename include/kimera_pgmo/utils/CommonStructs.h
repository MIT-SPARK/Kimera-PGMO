/**
 * @file   CommonStructs.h
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#pragma once

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

#include <pcl/PolygonMesh.h>

namespace kimera_pgmo {

typedef std::uint64_t Vertex;
typedef std::vector<Vertex> Vertices;
typedef std::map<Vertex, Vertices> Edges;
typedef std::pair<Vertex, Vertex> Edge;

class Graph {
 public:
  inline Vertices getValence(const Vertex& v) const { return edges_.at(v); }
  inline Vertices getVertices() const { return vertices_; }

  std::vector<Edge> getEdges() const;

  bool addEdge(const Edge& e, bool check = true);
  void addVertex(const Vertex& v);
  void addEdgeAndVertices(const Edge& e);
  bool combineGraph(const Graph& new_graph);
  bool createFromPclMesh(const pcl::PolygonMesh& mesh);
  bool createFromPclMeshBidirection(const pcl::PolygonMesh& mesh);
  std::vector<Edge> addPointsAndSurfaces(
      const std::vector<size_t>& vertices,
      const std::vector<pcl::Vertices>& polygons);
  void print(std::string header) const;

 private:
  Vertices vertices_;
  Edges edges_;
  Vertex max_vertex_ = 0;
};

typedef std::shared_ptr<Graph> GraphPtr;

}  // namespace kimera_pgmo
