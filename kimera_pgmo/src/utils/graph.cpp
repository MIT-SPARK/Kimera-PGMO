
/**
 * @file   common_structs.cpp
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#include "kimera_pgmo/utils/graph.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <algorithm>

#include "kimera_pgmo/utils/common_structs.h"

namespace kimera_pgmo {

using pose_graph_tools::PoseGraph;
using PclFaces = std::vector<pcl::Vertices>;
using GraphEdges = std::vector<Graph::Edge>;

std::vector<Graph::Edge> Graph::getEdges() const {
  std::vector<Edge> edges;
  for (const auto& [v1, connections] : edges_) {
    for (const auto& v2 : connections) {
      edges.push_back({v1, v2});
    }
  }

  return edges;
}

void Graph::addVertex(const Node& v) {
  if (v > max_vertex_) {
    max_vertex_ = v;
  } else {
    for (const auto& vertex : vertices_) {
      if (v == vertex) {
        return;
      }
    }
  }

  vertices_.push_back(v);
  edges_[v] = {};
}

void Graph::addEdgeAndVertices(const Edge& e) {
  addVertex(e.first);
  addVertex(e.second);
  addEdge(e);
}

bool Graph::addEdge(const Edge& e, bool check) {
  // Push edge
  if (!check) {
    edges_[e.first].push_back(e.second);
    return true;
  }

  auto iter = edges_.find(e.first);
  if (iter == edges_.end()) {
    edges_[e.first] = {e.second};
    return true;
  }

  auto iter2 = std::find(iter->second.begin(), iter->second.end(), e.second);
  if (iter2 == iter->second.end()) {
    iter->second.push_back(e.second);
    return true;
  }

  return false;
}

GraphEdges Graph::addPointsAndSurfaces(const std::vector<size_t>& vertices,
                                       const PclFaces& polygons,
                                       bool bidirectional) {
  // return the new edges
  for (const auto& v : vertices) {
    addVertex(v);
  }

  std::vector<Edge> new_edges;
  for (const auto& polygon : polygons) {
    for (size_t i = 0; i < polygon.vertices.size(); i++) {
      size_t i_next = (i + 1) % polygon.vertices.size();
      Edge e1(polygon.vertices[i], polygon.vertices[i_next]);
      if (addEdge(e1, true)) {
        new_edges.push_back(e1);
      }

      if (!bidirectional) {
        continue;
      }

      Edge e2(polygon.vertices[i_next], polygon.vertices[i]);
      if (addEdge(e2, true)) {
        new_edges.push_back(e2);
      }
    }
  }

  return new_edges;
}

bool Graph::combineGraph(const Graph& new_graph) {
  const auto new_edges = new_graph.getEdges();
  for (const auto& e : new_edges) {
    addEdgeAndVertices(e);
  }

  return true;
}

void Graph::print(std::string header) const {
  std::cout << header << "\n";
  std::cout << "vertices: \n";
  for (const auto& v : vertices_) {
    std::cout << v << " ";
  }

  std::cout << "\nedges: \n";
  Edges::const_iterator iter;
  for (const auto& [v1, connections] : edges_) {
    for (const auto& v2 : connections) {
      std::cout << v1 << "-->" << v2 << " ";
    }
  }

  std::cout << std::endl;
}

Eigen::Vector3d PclToEigen(const pcl::PointXYZRGBA& p) { return {p.x, p.y, p.z}; }

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
PoseGraph::Ptr makePoseGraph(int robot_id,
                             double time_in_sec,
                             const std::vector<Graph::Edge>& new_edges,
                             const std::vector<size_t>& new_indices,
                             const pcl::PointCloud<pcl::PointXYZRGBA>& vertices) {
  // Create message
  auto pose_graph = std::make_shared<PoseGraph>();
  pose_graph->stamp_ns = stampFromSec(time_in_sec);

  // Encode the edges as factors
  pose_graph->edges.reserve(new_edges.size());
  for (auto&& [from_node, to_node] : new_edges) {
    auto& pg_edge = pose_graph->edges.emplace_back();
    pg_edge.key_from = from_node;
    pg_edge.key_to = to_node;
    pg_edge.robot_from = robot_id;
    pg_edge.robot_to = robot_id;
    pg_edge.type = pose_graph_tools::PoseGraphEdge::MESH;
    pg_edge.stamp_ns = pose_graph->stamp_ns;

    const auto p_from = PclToEigen(vertices.at(from_node));
    const auto p_to = PclToEigen(vertices.at(to_node));
    pg_edge.pose = Eigen::Translation<double, 3>(p_to - p_from);
  }

  // Encode the new vertices as nodes
  pose_graph->nodes.reserve(new_indices.size());
  for (const auto& idx : new_indices) {
    auto& pg_node = pose_graph->nodes.emplace_back();
    pg_node.stamp_ns = pose_graph->stamp_ns;
    pg_node.robot_id = robot_id;
    pg_node.key = idx;
    pg_node.pose = Eigen::Translation<double, 3>(PclToEigen(vertices.at(idx)));
  }

  return pose_graph;
}

}  // namespace kimera_pgmo
