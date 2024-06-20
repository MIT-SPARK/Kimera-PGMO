/**
 * @file   common_structs.cpp
 * @brief  Some common structs and types used in library
 * @author Yun Chang
 */

#include "kimera_pgmo/utils/common_structs.h"

#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <chrono>
#include <numeric>

namespace kimera_pgmo {

using pose_graph_tools::PoseGraph;

// Timestamps
Timestamp stampFromSec(double sec) {
  auto t = std::chrono::duration<double>(sec);
  return std::chrono::duration_cast<std::chrono::nanoseconds>(t).count();
}

double stampToSec(Timestamp stamp) {
  auto t = std::chrono::nanoseconds(stamp);
  return std::chrono::duration<double>(t).count();
}

//// Graph Class
std::vector<Edge> Graph::getEdges() const {
  std::vector<Edge> edges;
  for (auto it = edges_.begin(); it != edges_.end(); ++it) {
    Vertex v1 = it->first;
    for (const Vertex& v2 : it->second) {
      Edge e(v1, v2);
      edges.push_back(e);
    }
  }
  return edges;
}

void Graph::addVertex(const Vertex& v) {
  if (v > max_vertex_) {
    max_vertex_ = v;
  } else {
    for (const Vertex& vertex : vertices_) {
      if (v == vertex) return;
    }
  }
  vertices_.push_back(v);
  edges_[v] = Vertices();
}

void Graph::addEdgeAndVertices(const Edge& e) {
  Vertex v1 = e.first;
  Vertex v2 = e.second;
  addVertex(v1);
  addVertex(v2);
  addEdge(e);
}

bool Graph::addEdge(const Edge& e, bool check) {
  // Push edge
  if (!check) {
    edges_[e.first].push_back(e.second);
    return true;
  }
  Edges::iterator iter = edges_.find(e.first);
  if (iter == edges_.end()) {
    edges_[e.first] = Vertices{e.second};
    return true;
  } else {
    std::vector<Vertex>::iterator iter2;
    iter2 = std::find(edges_[e.first].begin(), edges_[e.first].end(), e.second);
    if (iter2 == edges_[e.first].end()) {
      edges_[e.first].push_back(e.second);
      return true;
    }
  }
  return false;
}

bool Graph::createFromPclMesh(const pcl::PolygonMesh& mesh) {
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  const size_t n = cloud.points.size();
  for (Vertex v = 0; v < n; v++) {
    addVertex(v);
  }
  for (const pcl::Vertices& polygon : mesh.polygons) {
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
  const size_t& n = cloud.points.size();
  for (Vertex v = 0; v < n; v++) {
    addVertex(v);
  }
  for (const pcl::Vertices& polygon : mesh.polygons) {
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

std::vector<Edge> Graph::addPointsAndSurfaces(
    const std::vector<size_t>& vertices,
    const std::vector<pcl::Vertices>& polygons) {
  // return the new edges
  for (const Vertex& v : vertices) {
    addVertex(v);
  }

  std::vector<Edge> new_edges;
  for (const pcl::Vertices& polygon : polygons) {
    for (size_t i = 0; i < polygon.vertices.size(); i++) {
      size_t i_next = (i + 1) % polygon.vertices.size();
      Edge e1(polygon.vertices[i], polygon.vertices[i_next]);
      if (addEdge(e1, true)) new_edges.push_back(e1);
      Edge e2(polygon.vertices[i_next], polygon.vertices[i]);
      if (addEdge(e2, true)) new_edges.push_back(e2);
    }
  }
  return new_edges;
}

bool Graph::combineGraph(const Graph& new_graph) {
  const std::vector<Edge>& new_edges = new_graph.getEdges();

  for (const Edge& e : new_edges) {
    addEdgeAndVertices(e);
  }

  return true;
}

void Graph::print(std::string header) const {
  std::cout << header << "\n";
  std::cout << "vertices: \n";
  for (Vertex v : vertices_) {
    std::cout << v << " ";
  }
  std::cout << std::endl;
  std::cout << "edges: \n";
  Edges::const_iterator iter;
  for (iter = edges_.begin(); iter != edges_.end(); iter++) {
    for (Vertex v : iter->second) {
      std::cout << iter->first << "-->" << v << " ";
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
                             const std::vector<Edge>& new_edges,
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
