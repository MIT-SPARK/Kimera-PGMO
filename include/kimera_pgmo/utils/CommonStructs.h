/**
 * @file   CommonStructs.h
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#pragma once

#include <cstdint>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>
#include <map>

#include <voxblox/core/block_hash.h>
#include <voxblox/core/common.h>

#include <pcl/PolygonMesh.h>

namespace kimera_pgmo {

typedef std::uint64_t Vertex;
typedef std::vector<Vertex> Vertices;
typedef std::map<Vertex, Vertices> Edges;
typedef std::pair<Vertex, Vertex> Edge;

typedef std::unordered_map<size_t, size_t> IndexMapping;
typedef voxblox::AnyIndexHashMapType<IndexMapping>::type VoxbloxIndexMapping;
typedef std::pair<voxblox::BlockIndex, IndexMapping> VoxbloxIndexPair;
typedef std::pair<voxblox::BlockIndex, size_t> VoxbloxBlockIndexPair;

enum class ProcessPoseGraphStatus {
  EMPTY,
  INVALID,
  MISSING,
  UNKNOWN,
  DUPLICATE,
  SUCCESS,
  LC_MISSING_NODES
};
enum class ProcessMeshGraphStatus {
  EMPTY,
  INVALID,
  UNKNOWN,
  DUPLICATE,
  SUCCESS,
  WAITING
};

class Graph {
 public:
  /*! \brief Get the connected vertices of a vertex
   *  - v: vertex to query
   *  - output the connected vertices
   */
  inline Vertices getValence(const Vertex& v) const { return edges_.at(v); }

  /*! \brief Get the vertices of in th graph
   *  - outputs the vertices
   */
  inline Vertices getVertices() const { return vertices_; }

  /*! \brief Get the edges in the graph
   *  -outputs the edges
   */
  std::vector<Edge> getEdges() const;

  /*! \brief Add single edge
   *  - e: edge to add
   *  - check: whether to check if edge exists before adding to avoid
   * duplication
   */
  bool addEdge(const Edge& e, bool check = true);

  /*! \brief Add vertex to graph
   *  - v: vertex to add
   */
  void addVertex(const Vertex& v);

  /*! \brief Add and edge along with the two vertices of the edge
   *  - e: edge to be added
   */
  void addEdgeAndVertices(const Edge& e);

  /*! \brief Add the edges and vertices of another graph to graph
   * new_graph: graph to be added
   */
  bool combineGraph(const Graph& new_graph);

  /*! \brief Create graph from a PolygonMesh with the mesh vertices as the graph
   * vertices and the sides of the mesh surfaces as edges (according to right
   * hand rule)
   *  - mesh: pcl PolygonMesh mesh
   */
  bool createFromPclMesh(const pcl::PolygonMesh& mesh);

  /*! \brief Create graph from a PolygonMesh with the mesh vertices as the graph
   * vertices and the sides of the mesh surfaces as bidirectional edges
   *  - mesh: pcl PolygonMesh mesh
   */
  bool createFromPclMeshBidirection(const pcl::PolygonMesh& mesh);

  /*! \brief Add new mesh vertices and surfaces and add these new mesh vertices
   * as graph vertices and sides of the new surfaces as bidirectional edges
   *  - vertices: mesh vertices
   *  - polygons: mesh surfaces
   */
  std::vector<Edge> addPointsAndSurfaces(
      const std::vector<size_t>& vertices,
      const std::vector<pcl::Vertices>& polygons);

  /*! \brief Print graph
   *  - header: label for output
   */
  void print(std::string header) const;

 private:
  Vertices vertices_;
  Edges edges_;
  Vertex max_vertex_ = 0;
};

typedef std::shared_ptr<Graph> GraphPtr;

}  // namespace kimera_pgmo
