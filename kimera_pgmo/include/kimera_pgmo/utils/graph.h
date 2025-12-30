/**
 * @file   common_structs.h
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#pragma once
#include <pcl/Vertices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pose_graph_tools/pose_graph.h>

#include <cstdint>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace kimera_pgmo {

class Graph {
 public:
  using Node = uint64_t;
  using Nodes = std::vector<Node>;
  using Edges = std::map<Node, Nodes>;
  using Edge = std::pair<Node, Node>;

  //! \brief Get the connected vertices of a vertex
  inline Nodes getValence(const Node& v) const { return edges_.at(v); }

  //! \brief Get the vertices of in th graph
  inline Nodes getVertices() const { return vertices_; }

  //! \brief Get the edges in the graph
  std::vector<Edge> getEdges() const;

  //! \brief Add single edge and optionally check for duplication
  bool addEdge(const Edge& e, bool check = true);

  //! \brief Add vertex to graph
  void addVertex(const Node& v);

  //! \brief Add and edge along with the two vertices of the edge
  void addEdgeAndVertices(const Edge& e);

  //! \brief Add the edges and vertices of another graph to graph
  bool combineGraph(const Graph& new_graph);

  /*! \brief Add new mesh vertices and surfaces and add these new mesh vertices
   * as graph vertices and sides of the new surfaces as bidirectional edges
   */
  std::vector<Edge> addPointsAndSurfaces(const std::vector<size_t>& vertices,
                                         const std::vector<pcl::Vertices>& polygons,
                                         bool bidirectional = true);

  //! \brief Print graph (with optional header
  void print(std::string header) const;

 private:
  Nodes vertices_;
  Edges edges_;
  Node max_vertex_ = 0;
};

/*! \brief Publish the factors corresponding to the new edges added to the
 * simplified mesh / deformation graph and also the initial values (positions
 * of the new vertices added to the simplified mesh)
 *  - new_edges: new edges of type Edge (std::pair<Node, Node>)
 *  - new_indices: new vertices of type Vertex
 *  - graph_vertices: deformation graph vertices
 *  - header: current mesh header
 *  - robot_id: robot for the deformation graph
 *  returns: published pose graph
 */
pose_graph_tools::PoseGraph::Ptr makePoseGraph(
    int robot_id,
    double time_in_sec,
    const std::vector<Graph::Edge>& new_edges,
    const std::vector<size_t>& new_indices,
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices);

}  // namespace kimera_pgmo
