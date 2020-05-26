/**
 * @file   testGraph.cpp
 * @brief  Unit-tests for the graph structure
 * @author Yun Chang
 */
#include <algorithm>
#include <numeric>
#include "gtest/gtest.h"

#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

#include "mesher_mapper/CommonStructs.h"
#include "test_config.h"

namespace mesher_mapper {

pcl::PolygonMesh createSimpleMesh() {
  // Create simple pcl mesh
  pcl::PolygonMesh mesh;

  pcl::PointCloud<pcl::PointXYZ> ptcld;
  ptcld.points.push_back(pcl::PointXYZ(0, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(1, 0, 0));
  ptcld.points.push_back(pcl::PointXYZ(0, 1, 0));
  ptcld.points.push_back(pcl::PointXYZ(1, 1, 0));
  ptcld.points.push_back(pcl::PointXYZ(0, 0, 1));
  pcl::toPCLPointCloud2(ptcld, mesh.cloud);

  pcl::Vertices tri_1, tri_2, tri_3, tri_4;
  tri_1.vertices = std::vector<uint>{0, 1, 2};
  tri_2.vertices = std::vector<uint>{1, 3, 2};
  tri_3.vertices = std::vector<uint>{0, 1, 4};
  tri_4.vertices = std::vector<uint>{0, 4, 2};
  mesh.polygons = std::vector<pcl::Vertices>{tri_1, tri_2, tri_3, tri_4};

  return mesh;
}

TEST(Graph, createFromPclMesh) {
  pcl::PolygonMesh mesh = createSimpleMesh();

  Graph new_graph;
  new_graph.createFromPclMesh(mesh);

  Vertices expected_vertices(5);
  std::iota(std::begin(expected_vertices), std::end(expected_vertices), 0);
  EXPECT_EQ(expected_vertices, new_graph.getVertices());
  std::vector<Edge> new_edges = new_graph.getEdges();
  EXPECT_EQ(Edge(0, 1), new_edges[0]);
  EXPECT_EQ(Edge(0, 4), new_edges[1]);
  EXPECT_EQ(Edge(4, 2), new_edges[new_edges.size() - 1]);
  Vertices valences_2 = Vertices{0, 1};
  EXPECT_EQ(valences_2, new_graph.getValence(2));
  Vertices valences_3 = Vertices{2};
  EXPECT_EQ(valences_3, new_graph.getValence(3));
  Vertices valences_4 = Vertices{0, 2};
  EXPECT_EQ(valences_4, new_graph.getValence(4));
}

TEST(Graph, createFromPclMeshBidirection) {
  pcl::PolygonMesh mesh = createSimpleMesh();

  Graph new_graph;
  new_graph.createFromPclMeshBidirection(mesh);

  Vertices expected_vertices(5);
  std::iota(std::begin(expected_vertices), std::end(expected_vertices), 0);
  EXPECT_EQ(expected_vertices, new_graph.getVertices());
  std::vector<Edge> new_edges = new_graph.getEdges();
  EXPECT_EQ(Edge(0, 1), new_edges[0]);
  EXPECT_EQ(Edge(0, 2), new_edges[1]);
  EXPECT_EQ(Edge(4, 2), new_edges[new_edges.size() - 1]);

  Vertices valences_2 = Vertices{0, 1, 3, 4};
  Vertices actual_2 = new_graph.getValence(2);
  std::sort(actual_2.begin(), actual_2.end());
  EXPECT_EQ(valences_2, actual_2);

  Vertices valences_3 = Vertices{1, 2};
  Vertices actual_3 = new_graph.getValence(3);
  std::sort(actual_3.begin(), actual_3.end());
  EXPECT_EQ(valences_3, actual_3);

  Vertices valences_4 = Vertices{0, 1, 2};
  Vertices actual_4 = new_graph.getValence(4);
  std::sort(actual_4.begin(), actual_4.end());
  EXPECT_EQ(valences_4, actual_4);
}

TEST(Graph, addEdgeAndVertices) {
  Graph new_graph;
  new_graph.addEdgeAndVertices(Edge(0, 1));
  Vertices new_vertices = Vertices{0, 1};
  EXPECT_EQ(new_vertices, new_graph.getVertices());
  Vertices valence_0 = Vertices{1};
  Vertices valence_1;
  EXPECT_EQ(valence_0, new_graph.getValence(0));
  std::vector<Edge> new_edges = std::vector<Edge>{Edge(0, 1)};
  EXPECT_EQ(new_edges, new_graph.getEdges());
  // Add two more edges
  new_graph.addEdgeAndVertices(Edge(1, 0));
  new_graph.addEdgeAndVertices(Edge(0, 2));
  new_vertices.push_back(2);
  new_edges = std::vector<Edge>{Edge(0, 1), Edge(0, 2), Edge(1, 0)};
  EXPECT_EQ(new_edges, new_graph.getEdges());
  EXPECT_EQ(new_vertices, new_graph.getVertices());
  valence_0.push_back(2);
  EXPECT_EQ(valence_0, new_graph.getValence(0));
}

TEST(Graph, addEdge) {
  pcl::PolygonMesh mesh = createSimpleMesh();

  Graph new_graph;
  new_graph.createFromPclMesh(mesh);

  new_graph.addEdge(Edge(3, 1));
  new_graph.addEdge(Edge(3, 4));

  Vertices expected_vertices(5);
  std::iota(std::begin(expected_vertices), std::end(expected_vertices), 0);
  EXPECT_EQ(expected_vertices, new_graph.getVertices());

  Vertices valences_2 = Vertices{0, 1};
  EXPECT_EQ(valences_2, new_graph.getValence(2));
  Vertices valences_3 = Vertices{2, 1, 4};
  EXPECT_EQ(valences_3, new_graph.getValence(3));
  Vertices valences_4 = Vertices{0, 2};
  EXPECT_EQ(valences_4, new_graph.getValence(4));
}

TEST(Graph, combineGraph) {
  Graph graph_1, graph_2;

  graph_1.addEdgeAndVertices(Edge(0, 1));
  graph_1.addEdgeAndVertices(Edge(1, 2));
  graph_1.addEdgeAndVertices(Edge(2, 0));

  graph_2.addEdgeAndVertices(Edge(1, 3));
  graph_2.addEdgeAndVertices(Edge(3, 2));
  graph_2.addEdgeAndVertices(Edge(2, 1));

  graph_1.combineGraph(graph_2);

  Vertices expected_vertices(4);
  std::iota(std::begin(expected_vertices), std::end(expected_vertices), 0);
  EXPECT_EQ(expected_vertices, graph_1.getVertices());

  EXPECT_EQ(Edge(0, 1), graph_1.getEdges()[0]);
  EXPECT_EQ(Edge(3, 2), graph_1.getEdges()[5]);
}

}  // namespace mesher_mapper