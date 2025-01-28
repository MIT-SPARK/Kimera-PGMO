/**
 * @file   common_structs.h
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#pragma once
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pose_graph_tools/pose_graph.h>

#include <cstdint>
#include <map>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

using Vertex = uint64_t;
using Vertices = std::vector<Vertex>;
using Edges = std::map<Vertex, Vertices>;
using Edge = std::pair<Vertex, Vertex>;

using traits::Timestamp;
Timestamp stampFromSec(double sec);
double stampToSec(Timestamp stamp);

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
  std::vector<Edge> addPointsAndSurfaces(const std::vector<size_t>& vertices,
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

using GraphPtr = std::shared_ptr<Graph>;

/*! \brief Publish the factors corresponding to the new edges added to the
 * simplified mesh / deformation graph and also the initial values (positions
 * of the new vertices added to the simplified mesh)
 *  - new_edges: new edges of type Edge (std::pair<Vertex, Vertex>)
 *  - new_indices: new vertices of type Vertex
 *  - graph_vertices: deformation graph vertices
 *  - header: current mesh header
 *  - robot_id: robot for the deformation graph
 *  returns: published pose graph
 */
pose_graph_tools::PoseGraph::Ptr makePoseGraph(
    int robot_id,
    double time_in_sec,
    const std::vector<Edge>& new_edges,
    const std::vector<size_t>& new_indices,
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices);

}  // namespace kimera_pgmo
