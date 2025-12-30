/**
 * @file   test_graph.cpp
 * @brief  Unit-tests for the graph structure
 * @author Yun Chang
 */
#include <gtest/gtest.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>

#include <numeric>

#include "kimera_pgmo/utils/graph.h"
#include "pgmo_fixtures.h"

namespace kimera_pgmo {

TEST(TestGraph, addEdgeAndVertices) {
  Graph new_graph;
  new_graph.addEdgeAndVertices({0, 1});

  Graph::Nodes new_vertices{0, 1};
  EXPECT_EQ(new_vertices, new_graph.getVertices());

  Graph::Nodes valence_0{1};
  EXPECT_EQ(valence_0, new_graph.getValence(0));

  std::vector<Graph::Edge> new_edges{{0, 1}};
  EXPECT_EQ(new_edges, new_graph.getEdges());

  // Add two more edges
  new_graph.addEdgeAndVertices({1, 0});
  new_graph.addEdgeAndVertices({0, 2});

  new_vertices.push_back(2);
  EXPECT_EQ(new_vertices, new_graph.getVertices());

  new_edges = std::vector<Graph::Edge>{{0, 1}, {0, 2}, {1, 0}};
  EXPECT_EQ(new_edges, new_graph.getEdges());

  valence_0.push_back(2);
  EXPECT_EQ(valence_0, new_graph.getValence(0));
}

TEST(TestGraph, addEdge) {
  auto mesh = test::createSimpleMesh();

  const auto num_nodes = mesh.cloud.width * mesh.cloud.height;
  std::vector<size_t> new_nodes(num_nodes);
  std::iota(new_nodes.begin(), new_nodes.end(), 0);

  Graph new_graph;
  new_graph.addPointsAndSurfaces(new_nodes, mesh.polygons, false);

  new_graph.addEdge({3, 1});
  new_graph.addEdge({3, 4});

  Graph::Nodes expected_vertices(5);
  std::iota(std::begin(expected_vertices), std::end(expected_vertices), 0);
  EXPECT_EQ(expected_vertices, new_graph.getVertices());

  Graph::Nodes valences_2{0, 1};
  EXPECT_EQ(valences_2, new_graph.getValence(2));
  Graph::Nodes valences_3{2, 1, 4};
  EXPECT_EQ(valences_3, new_graph.getValence(3));
  Graph::Nodes valences_4{0, 2};
  EXPECT_EQ(valences_4, new_graph.getValence(4));
}

TEST(TestGraph, combineGraph) {
  Graph graph_1, graph_2;

  graph_1.addEdgeAndVertices({0, 1});
  graph_1.addEdgeAndVertices({1, 2});
  graph_1.addEdgeAndVertices({2, 0});

  graph_2.addEdgeAndVertices({1, 3});
  graph_2.addEdgeAndVertices({3, 2});
  graph_2.addEdgeAndVertices({2, 1});

  graph_1.combineGraph(graph_2);

  Graph::Nodes expected_vertices(4);
  std::iota(std::begin(expected_vertices), std::end(expected_vertices), 0);
  EXPECT_EQ(expected_vertices, graph_1.getVertices());

  EXPECT_EQ(Graph::Edge(0, 1), graph_1.getEdges()[0]);
  EXPECT_EQ(Graph::Edge(3, 2), graph_1.getEdges()[5]);
}

TEST(TestGraph, addPointsAndSurfaces) {
  Graph graph;

  graph.addEdgeAndVertices({0, 1});
  graph.addEdgeAndVertices({1, 0});
  graph.addEdgeAndVertices({1, 2});
  graph.addEdgeAndVertices({2, 1});
  graph.addEdgeAndVertices({2, 0});
  graph.addEdgeAndVertices({0, 2});

  std::vector<size_t> new_indices = {3, 4};
  std::vector<pcl::Vertices> new_polygons{{{2, 3, 4}}, {{0, 1, 4}}};
  const auto new_edges = graph.addPointsAndSurfaces(new_indices, new_polygons);

  EXPECT_EQ(10u, new_edges.size());
  EXPECT_EQ(Graph::Edge(2, 3), new_edges[0]);
  EXPECT_EQ(Graph::Edge(0, 4), new_edges[9]);

  Graph::Nodes expected_vertices(5);
  std::iota(std::begin(expected_vertices), std::end(expected_vertices), 0);
  EXPECT_EQ(expected_vertices, graph.getVertices());

  EXPECT_EQ(16u, graph.getEdges().size());
  EXPECT_EQ(Graph::Edge(0, 1), graph.getEdges()[0]);
  EXPECT_EQ(Graph::Edge(4, 0), graph.getEdges()[15]);
}

}  // namespace kimera_pgmo
