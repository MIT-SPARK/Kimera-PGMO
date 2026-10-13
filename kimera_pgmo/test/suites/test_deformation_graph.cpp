/**
 * @file   test_deformation_graph.cpp
 * @brief  Unit-tests for the deformation graph class
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <kimera_rpgo/rpgo.h>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/utils/common_structs.h"
#include "test_config.h"

namespace kimera_pgmo {
namespace {

gtsam::Symbol operator"" _id(const char* str, size_t size) {
  if (size < 1) {
    throw std::domain_error("invalid literal: must have at least two characters");
  }

  char prefix = str[0];
  std::string number(str + 1, size - 1);
  size_t index = std::stoull(number);
  return gtsam::Symbol(prefix, index);
}

template <typename T, typename Ptr>
const T* cast_factor(const Ptr& ptr, bool should_throw = false) {
  if (!ptr) {
    if (should_throw) {
      throw std::runtime_error("bad pointer");
    }
    return nullptr;
  }

  auto cast = dynamic_cast<const T*>(ptr.get());
  if (!cast && should_throw) {
    throw std::runtime_error("bad cast");
  }

  return cast;
}

void ComparePointcloud(const std::vector<traits::Vertex>& cloud1,
                       const std::vector<traits::Vertex>& cloud2,
                       double precision = 0.0) {
  if (cloud1.size() != cloud2.size()) {
    throw std::runtime_error("cloud sizes don't match");
  }

  for (size_t i = 0; i < cloud1.size(); i++) {
    EXPECT_TRUE(cloud1[i].pos.isApprox(cloud2[i].pos, precision));
  }
}

void SetUpDeformationGraph(DeformationGraph* graph) {
  gtsam::Values mesh_nodes;
  std::unordered_map<gtsam::Key, Timestamp> mesh_node_stamps;
  mesh_nodes.insert("v0"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 0.0, 0.0)));
  mesh_node_stamps["v0"_id] = 0;
  mesh_nodes.insert("v1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1.0, 0.0, 0.0)));
  mesh_node_stamps["v1"_id] = 0;
  mesh_nodes.insert("v2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0.0, 1.0, 0.0)));
  mesh_node_stamps["v2"_id] = 0;

  std::vector<std::pair<gtsam::Key, gtsam::Key>> mesh_edges;
  mesh_edges.push_back({"v0"_id, "v1"_id});
  mesh_edges.push_back({"v1"_id, "v0"_id});
  mesh_edges.push_back({"v1"_id, "v2"_id});
  mesh_edges.push_back({"v2"_id, "v1"_id});
  mesh_edges.push_back({"v2"_id, "v0"_id});
  mesh_edges.push_back({"v0"_id, "v2"_id});

  std::vector<size_t> added_node_indices;
  std::vector<Timestamp> added_node_stamps;
  graph->processNewMeshEdgesAndNodes(mesh_edges,
                                     mesh_nodes,
                                     mesh_node_stamps,
                                     &added_node_indices,
                                     &added_node_stamps);
}

traits::Vertex makePoint(float x, float y, float z, Timestamp stamp) {
  traits::VertexTraits traits;
  traits.properties.has_stamp = true;
  traits.stamp = stamp;
  return {traits::Pos(x, y, z), traits};
}

std::vector<traits::Vertex> makeTriangle() {
  std::vector<traits::Vertex> points;
  points.push_back(makePoint(0.0, 0.0, 0.0, 0));
  points.push_back(makePoint(1.0, 0.0, 0.0, 0));
  points.push_back(makePoint(0.0, 1.0, 0.0, 0));
  return points;
}

std::vector<traits::Vertex> makePartialCube() {
  std::vector<traits::Vertex> points;
  points.push_back(makePoint(0.0, 0.0, 0.0, 0));
  points.push_back(makePoint(1.0, 0.0, 0.0, 0));
  points.push_back(makePoint(0.0, 1.0, 0.0, 0));
  points.push_back(makePoint(1.0, 1.0, 0.0, 0));
  points.push_back(makePoint(0.0, 0.0, 1.0, 0));
  return points;
}

std::vector<traits::Vertex> makeCube() {
  std::vector<traits::Vertex> points;
  points.push_back(makePoint(0.0, 0.0, 0.0, 0));
  points.push_back(makePoint(1.0, 0.0, 0.0, 0));
  points.push_back(makePoint(0.0, 1.0, 0.0, 0));
  points.push_back(makePoint(0.0, 0.0, 1.0, 0));
  points.push_back(makePoint(1.0, 1.0, 0.0, 0));
  points.push_back(makePoint(1.0, 0.0, 1.0, 0));
  points.push_back(makePoint(0.0, 1.0, 1.0, 0));
  points.push_back(makePoint(1.0, 1.0, 1.0, 0));
  return points;
}

std::vector<traits::Vertex> deformVertices(const DeformationGraph& graph,
                                           const std::vector<traits::Vertex>& original,
                                           char prefix,
                                           size_t k = 4,
                                           double tol_t = 10.0) {
  std::vector<traits::Vertex> new_points = original;
  graph.deformAllPoints(new_points, original, prefix, k, tol_t);
  return new_points;
}

void OptimizeOnce(DeformationGraph& deformation_graph) {
  kimera_rpgo::RpgoConfig pgo_config;
  pgo_config.solver_config.setLeastSquaresParamsDefault();
  kimera_rpgo::Rpgo pgo(pgo_config);

  auto factors = *deformation_graph.getFactors();
  auto temp_factors = *deformation_graph.getTempFactors();
  auto values = *deformation_graph.getValues();
  auto temp_values = *deformation_graph.getTempValues();
  pgo.addFactors(factors);
  pgo.addValues(values);
  pgo.addFactors(temp_factors);
  pgo.addValues(temp_values);
  pgo.run();

  // Get estimates
  auto results = gtsam::Values(values);
  auto temp_results = gtsam::Values(temp_values);
  auto rpgo_result = pgo.getResult();
  for (const auto& key_val : rpgo_result) {
    if (results.exists(key_val.key)) {
      results.update(key_val.key, key_val.value);
    }
    if (temp_results.exists(key_val.key)) {
      temp_results.update(key_val.key, key_val.value);
    }
  }

  deformation_graph.updateValues(results);
  deformation_graph.updateTempValues(temp_results);
}

}  // namespace

TEST(TestDeformationGraph, processNewMeshEdgesAndNodes) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  // Check sizes
  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, graph.getInitialPositionVertex('v', 2).y());

  // Check that the factors are added
  auto values = graph.getValues();
  auto factors = graph.getFactors();

  EXPECT_EQ(6u, factors->size());
  EXPECT_EQ(3u, values->size());
}

TEST(TestDeformationGraph, reconstructMesh) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  auto original = makePartialCube();

  // First try deform with k = 1, should not change
  auto new_points = deformVertices(graph, original, 'v', 1);
  ASSERT_EQ(5, new_points.size());
  EXPECT_EQ(1, new_points[1].pos.x());
  EXPECT_EQ(1, new_points[2].pos.y());
  EXPECT_EQ(1, new_points[4].pos.z());

  // Try with k = 2
  new_points = deformVertices(graph, original, 'v', 2);
  ASSERT_EQ(5, new_points.size());
  EXPECT_EQ(1, new_points[1].pos.x());
  EXPECT_EQ(1, new_points[2].pos.y());
  EXPECT_EQ(1, new_points[4].pos.z());
}

TEST(TestDeformationGraph, deformMeshTranslation) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  auto original = makePartialCube();

  // deform mesh
  gtsam::Pose3 distortion(gtsam::Rot3(), gtsam::Point3(1.5, 0.0, 0.0));
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> node_distortions;
  node_distortions.push_back({"v1"_id, distortion});
  graph.processNodeMeasurements(node_distortions, 1e-10);

  std::vector<traits::Vertex> expected;
  for (const auto& p : original) {
    auto& new_point = expected.emplace_back();
    new_point.pos = p.pos;
    new_point.pos.x() += 0.5;
  }

  // First try deform with k = 1, should not change
  OptimizeOnce(graph);

  auto result = deformVertices(graph, original, 'v', 1);
  ComparePointcloud(expected, result, 1e-6);

  // Try with k = 2
  result = deformVertices(graph, original, 'v', 2);
  ComparePointcloud(expected, result, 1e-6);
}

TEST(TestDeformationGraph, deformMesh) {
  const auto original = makeCube();
  std::vector<traits::Vertex> expected;
  for (const auto& p : original) {
    auto& new_point = expected.emplace_back();
    new_point.pos = p.pos;
    new_point.pos.x() -= 0.5;
  }

  // deform mesh
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  gtsam::Pose3 distortion(gtsam::Rot3(), gtsam::Point3(-0.5, 0.0, 0.0));
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> node_distortions;
  node_distortions.push_back({"v0"_id, distortion});
  graph.processNodeMeasurements(node_distortions, 1e-10);

  OptimizeOnce(graph);

  auto result = deformVertices(graph, original, 'v', 3);
  ComparePointcloud(expected, result, 1e-6);

  // deform mesh again
  gtsam::Pose3 distortion2(gtsam::Rot3(), gtsam::Point3(1.5, 0.0, 0.0));
  node_distortions.clear();
  node_distortions.push_back({"v1"_id, distortion2});
  graph.processNodeMeasurements(node_distortions, 1e-10);

  OptimizeOnce(graph);

  result = deformVertices(graph, original, 'v', 2);
  EXPECT_NEAR(-0.5, result[0].pos.x(), 0.001);
  EXPECT_NEAR(1.273, result[1].pos.x(), 0.001);
}

TEST(TestDeformationGraph, updateMesh) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  EXPECT_EQ(3, graph.getNumVertices());
  EXPECT_EQ(0, graph.getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, graph.getInitialPositionVertex('v', 2).y());

  // Check that the factors are added
  auto values = graph.getValues();
  auto factors = graph.getFactors();

  ASSERT_EQ(6u, factors->size());
  ASSERT_EQ(3u, values->size());
  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors->at(0)));
  DeformationEdgeFactor factor = *cast_factor<DeformationEdgeFactor>(factors->at(0));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor.measurement()));

  std::vector<uint64_t> new_node_valences{0, 2};
  graph.addNewNode(
      "a0"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), false);
  graph.processNodeValence("a0"_id, new_node_valences, 'v');

  // Check that the factors are added correctly
  values = graph.getValues();
  factors = graph.getFactors();

  ASSERT_EQ(10u, factors->size());
  ASSERT_EQ(4u, values->size());
  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors->at(9)));
  DeformationEdgeFactor factor6 = *cast_factor<DeformationEdgeFactor>(factors->at(6));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(-2, -2, -2), factor6.measurement()));
  EXPECT_EQ("a0"_id.key(), factor6.front());
  EXPECT_EQ("v0"_id.key(), factor6.back());

  std::vector<uint64_t> new_node_valences_2{2};
  graph.addNewNode(
      "a1"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)), false);
  graph.processNewBetween(
      "a0"_id, "a1"_id, gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(0, 1, 2)));
  graph.processNodeValence("a1"_id, new_node_valences_2, 'v');

  // Check that the factors are added
  values = graph.getValues();
  factors = graph.getFactors();

  ASSERT_EQ(13u, factors->size());
  ASSERT_EQ(5u, values->size());
  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors->at(12)));
  DeformationEdgeFactor factor12 = *cast_factor<DeformationEdgeFactor>(factors->at(12));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 2, 4), factor12.measurement()));
  EXPECT_EQ("v2"_id.key(), factor12.front());
  EXPECT_EQ("a1"_id.key(), factor12.back());
}

TEST(TestDeformationGraph, addNodeMeasurements) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  auto original = makeTriangle();

  std::vector<uint64_t> new_node_valences{0, 2};
  graph.addNewNode(
      "a0"_id, 0, gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2)), false);
  graph.addNewNode(
      "a1"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), false);
  graph.processNodeValence("a0"_id, new_node_valences, 'v');

  // Check factors added
  auto factors = graph.getFactors();

  // Add node measurement
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> measurements;
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      "a0"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2))));
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      "a1"_id, gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2))));

  graph.processNodeMeasurements(measurements);

  factors = graph.getFactors();

  EXPECT_EQ(12u, factors->size());
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(10)));
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(11)));

  OptimizeOnce(graph);

  auto result = deformVertices(graph, original, 'v', 1);
  EXPECT_NEAR(4, result[0].pos.x(), 0.001);
  EXPECT_NEAR(4, result[0].pos.y(), 0.001);
  EXPECT_NEAR(0, result[0].pos.z(), 0.001);
  EXPECT_NEAR(3, result[1].pos.x(), 0.001);
  EXPECT_NEAR(3, result[2].pos.y(), 0.001);
  EXPECT_NEAR(0, result[1].pos.z(), 0.001);
  EXPECT_NEAR(4, result[2].pos.x(), 0.001);
}

TEST(TestDeformationGraph, removePriorsWithPrefix) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  const auto original = makeTriangle();

  std::vector<uint64_t> new_node_valences{0, 2};
  graph.addNewNode(
      "a0"_id, 0, gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2)), false);
  graph.processNodeValence("a0"_id, new_node_valences, 'v');

  // Add node measurement
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> measurements;
  measurements.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
      "a0"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2))));

  graph.processNodeMeasurements(measurements);

  auto factors = graph.getFactors();

  EXPECT_EQ(11u, factors->size());
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(10)));

  OptimizeOnce(graph);

  auto result = deformVertices(graph, original, 'v', 1);
  EXPECT_NEAR(4, result[0].pos.x(), 0.001);
  EXPECT_NEAR(4, result[0].pos.y(), 0.001);
  EXPECT_NEAR(0, result[0].pos.z(), 0.001);
  EXPECT_NEAR(3, result[1].pos.x(), 0.001);
  EXPECT_NEAR(3, result[2].pos.y(), 0.001);
  EXPECT_NEAR(0, result[1].pos.z(), 0.001);
  EXPECT_NEAR(4, result[2].pos.x(), 0.001);

  // Up to here should be same as last test case
  // Remove priors
  graph.removePriorsWithPrefix('a');

  factors = graph.getFactors();
  EXPECT_EQ(10u, factors->size());

  // Add another prior to see if mesh reset
  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> priors;
  priors.push_back(
      {"a0"_id, gtsam::Pose3(gtsam::Rot3(0, 0, 0, 1), gtsam::Point3(2, 2, 2))});
  graph.processNodeMeasurements(priors);

  OptimizeOnce(graph);

  result = deformVertices(graph, original, 'v', 1);
  EXPECT_NEAR(0, result[0].pos.x(), 0.001);
  EXPECT_NEAR(0, result[0].pos.y(), 0.001);
  EXPECT_NEAR(0, result[0].pos.z(), 0.001);
  EXPECT_NEAR(1, result[1].pos.x(), 0.001);
  EXPECT_NEAR(1, result[2].pos.y(), 0.001);
  EXPECT_NEAR(0, result[1].pos.z(), 0.001);
  EXPECT_NEAR(0, result[2].pos.x(), 0.001);
}

TEST(TestDeformationGraph, processNewBetween) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  const auto original = makePartialCube();

  std::vector<uint64_t> new_node_valences{0, 2};
  graph.addNewNode(
      "a0"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), true);
  graph.addNewNode(
      "a1"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)), false);
  graph.processNewBetween(
      "a0"_id, "a1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)));
  graph.processNodeValence("a0"_id, new_node_valences, 'v');

  OptimizeOnce(graph);

  auto factors = graph.getFactors();
  auto values = graph.getValues();
  ASSERT_EQ(12u, factors->size());
  ASSERT_EQ(5u, values->size());

  // Check the prior factor
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(6)));
  gtsam::PriorFactor<gtsam::Pose3> prior_factor =
      *cast_factor<gtsam::PriorFactor<gtsam::Pose3>>(factors->at(6));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                                  prior_factor.prior()));
  EXPECT_EQ("a0"_id.key(), prior_factor.key());

  // Check the between factor
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(7)));
  gtsam::BetweenFactor<gtsam::Pose3> odom_factor =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(7));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)),
                                  odom_factor.measured()));
  EXPECT_EQ("a0"_id.key(), odom_factor.front());
  EXPECT_EQ("a1"_id.key(), odom_factor.back());

  std::vector<gtsam::Pose3> traj = graph.getTrajectory('a');
  EXPECT_EQ(2, traj.size());
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)),
                                  traj[0]));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)),
                                  traj[1]));

  graph.addNewNode(
      "a2"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)), false);
  graph.processNewBetween(
      "a1"_id, "a2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)));
  graph.processNewBetween(
      "a0"_id, "a2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)));

  std::vector<uint64_t> new_node_valences_2{2};
  graph.processNodeValence("a2"_id, new_node_valences_2, 'v');

  OptimizeOnce(graph);

  ASSERT_EQ(16u, factors->size());
  ASSERT_EQ(6u, values->size());

  // Check the between factor
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(12)));
  gtsam::BetweenFactor<gtsam::Pose3> new1 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(12));
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)), new1.measured()));
  EXPECT_EQ("a1"_id.key(), new1.front());
  EXPECT_EQ("a2"_id.key(), new1.back());

  // Check the between factor
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(13)));
  gtsam::BetweenFactor<gtsam::Pose3> new2 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3>>(factors->at(13));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                  new2.measured()));
  EXPECT_EQ("a0"_id.key(), new2.front());
  EXPECT_EQ("a2"_id.key(), new2.back());

  traj = graph.getTrajectory('a');
  ASSERT_EQ(3, traj.size());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.05, 2.05)), traj[2], 0.05));
}

TEST(TestDeformationGraph, addTemporary) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  std::vector<uint64_t> new_node_valences{0, 2};
  graph.addNewNode(
      "a0"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), true);
  graph.addNewNode(
      "a1"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)), false);
  graph.processNodeValence("a0"_id, new_node_valences, 'v');
  graph.processNewBetween(
      "a0"_id, "a1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)));

  graph.addNewNode(
      "a2"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)), false);
  graph.processNewBetween(
      "a1"_id, "a2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)));

  std::vector<uint64_t> new_node_valences_2{2};
  graph.processNodeValence("a2"_id, new_node_valences_2, 'v');

  // Check added factors
  auto factors = graph.getFactors();
  auto values = graph.getValues();
  auto temp_factors = graph.getTempFactors();
  auto temp_values = graph.getTempValues();

  EXPECT_EQ(15u, factors->size());
  EXPECT_EQ(6u, values->size());

  OptimizeOnce(graph);

  std::vector<gtsam::Pose3> traj = graph.getTrajectory('a');
  EXPECT_EQ(3, traj.size());

  // Add temporary nodes and edges
  graph.processNewTempNode("p0"_id, gtsam::Pose3(), false);
  graph.processNewTempNode(
      "p1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)), false);
  graph.processNewTempBetween(
      "p0"_id, "p1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)));
  std::vector<uint64_t> temp_node_valences{1, 2};
  graph.processNodeValence("p0"_id, temp_node_valences, 'v', 1e-4, true);

  // Check added factors
  OptimizeOnce(graph);

  // Check added factors
  EXPECT_EQ(15u, factors->size());
  EXPECT_EQ(6u, values->size());

  EXPECT_EQ(5u, temp_factors->size());
  EXPECT_EQ(2u, temp_values->size());

  traj = graph.getTrajectory('a');
  EXPECT_EQ(3, traj.size());

  graph.clearTemporaryStructures();

  OptimizeOnce(graph);

  EXPECT_EQ(0u, temp_factors->size());
  EXPECT_EQ(0u, temp_values->size());

  // Re-ad temporary nodes and edges
  graph.processNewTempNode("p0"_id, gtsam::Pose3(), false);
  std::vector<uint64_t> temp_node_valences_2{0, 1, 2};
  graph.processNodeValence("p0"_id, temp_node_valences_2, 'v', 1e-4, true);

  OptimizeOnce(graph);

  EXPECT_EQ(6u, temp_factors->size());
  EXPECT_EQ(1u, temp_values->size());
}

TEST(TestDeformationGraph, processNewTempNodesValences) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  NodeValenceInfoList info;
  for (size_t i = 0; i < 4; i++) {
    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0));
    info.push_back({'v', gtsam::Symbol('p', i), pose, {i}});
  }

  graph.processNewTempNodesValences(info, false);

  OptimizeOnce(graph);
  auto factors = graph.getFactors();
  auto values = graph.getValues();
  auto temp_factors = graph.getTempFactors();
  auto temp_values = graph.getTempValues();

  EXPECT_EQ(6u, factors->size());
  EXPECT_EQ(3u, values->size());

  for (size_t i = 0; i < 3; i++) {
    gtsam::Pose3 node_pose(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0));
    gtsam::Pose3 vertex_pose = values->at<gtsam::Pose3>(gtsam::Symbol('v', i));
    EXPECT_TRUE(gtsam::assert_equal(
        node_pose, temp_values->at<gtsam::Pose3>(gtsam::Symbol('p', i))));
    DeformationEdgeFactor factor1 =
        *cast_factor<DeformationEdgeFactor>(temp_factors->at(2 * i));
    DeformationEdgeFactor factor2 =
        *cast_factor<DeformationEdgeFactor>(temp_factors->at(2 * i + 1));
    EXPECT_TRUE(gtsam::assert_equal(
        node_pose.rotation().inverse().rotate(vertex_pose.translation() -
                                              node_pose.translation()),
        factor1.measurement()));
    EXPECT_TRUE(gtsam::assert_equal(
        vertex_pose.rotation().inverse().rotate(node_pose.translation() -
                                                vertex_pose.translation()),
        factor2.measurement()));
    EXPECT_EQ(gtsam::Symbol('p', i).key(), factor1.front());
    EXPECT_EQ(gtsam::Symbol('v', i).key(), factor1.back());
    EXPECT_EQ(gtsam::Symbol('v', i).key(), factor2.front());
    EXPECT_EQ(gtsam::Symbol('p', i).key(), factor2.back());
  }

  graph.clearTemporaryStructures();
  OptimizeOnce(graph);

  EXPECT_EQ(0u, temp_factors->size());
  EXPECT_EQ(0u, temp_values->size());
}

TEST(TestDeformationGraph, processNewTempEdges) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  NodeValenceInfoList info;
  for (size_t i = 0; i < 4; i++) {
    const gtsam::Pose3 pose(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 0, 0));
    info.push_back({'v', gtsam::Symbol('p', i), pose, {}});
  }

  graph.processNewTempNodesValences(info, false);

  pose_graph_tools::PoseGraph temp_edges;
  for (size_t i = 0; i < 3; i++) {
    pose_graph_tools::PoseGraphEdge edge;
    edge.key_from = gtsam::Symbol('p', i);
    edge.key_to = gtsam::Symbol('p', i + 1);
    temp_edges.edges.push_back(edge);
  }

  graph.processNewTempEdges(temp_edges);

  OptimizeOnce(graph);
  auto factors = graph.getFactors();
  auto values = graph.getValues();
  auto temp_factors = graph.getTempFactors();
  auto temp_values = graph.getTempValues();

  EXPECT_EQ(3u, values->size());
  EXPECT_EQ(6u, factors->size());
  EXPECT_EQ(4u, temp_values->size());
  EXPECT_EQ(3u, temp_factors->size());
}

TEST(TestDeformationGraph, saveAndLoad) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  std::vector<uint64_t> new_node_valences{0, 2};
  graph.addNewNode(
      "a0"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), true);
  graph.processNodeValence("a0"_id, new_node_valences, 'v');

  graph.addNewNode(
      "a1"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 3, 4)), false);
  graph.processNewBetween(
      "a0"_id, "a1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)));

  graph.addNewNode(
      "a2"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(3, 2.1, 2.1)), false);
  graph.processNewBetween(
      "a1"_id, "a2"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, -0.9, -1.9)));

  std::vector<uint64_t> new_node_valences_2{2};
  graph.processNodeValence("a2"_id, new_node_valences_2, 'v');

  // Add temporary nodes and edges
  graph.processNewTempNode("p0"_id, gtsam::Pose3(), false);
  graph.processNewTempNode(
      "p1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)), false);
  graph.processNewTempBetween(
      "p0"_id, "p1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 1)));
  std::vector<uint64_t> temp_node_valences{1, 2};
  graph.processNodeValence("p0"_id, temp_node_valences, 'v', 1e-4, true);

  auto values = graph.getValues();
  auto factors = graph.getFactors();
  auto temp_values = graph.getTempValues();
  auto temp_factors = graph.getTempFactors();

  EXPECT_EQ(15u, factors->size());
  EXPECT_EQ(6u, values->size());

  EXPECT_EQ(5u, temp_factors->size());
  EXPECT_EQ(2u, temp_values->size());
  const std::filesystem::path dgraph_path(std::string(DATASET_PATH) + "/graph.json");
  graph.save(dgraph_path);
  auto new_graph = DeformationGraph::load(dgraph_path);
  ASSERT_TRUE(new_graph);

  values = new_graph->getValues();
  factors = new_graph->getFactors();
  temp_values = new_graph->getTempValues();
  temp_factors = new_graph->getTempFactors();

  EXPECT_EQ(15u, factors->size());
  EXPECT_EQ(6u, values->size());

  EXPECT_EQ(5u, temp_factors->size());
  EXPECT_EQ(2u, temp_values->size());
  EXPECT_EQ(3, new_graph->getNumVertices());
  EXPECT_EQ(0, new_graph->getInitialPositionVertex('v', 0).x());
  EXPECT_EQ(1, new_graph->getInitialPositionVertex('v', 2).y());
}

TEST(TestDeformationGraph, processPoseGraph) {
  DeformationGraph graph;

  // Construct pose graph
  pose_graph_tools::PoseGraph pose_graph;
  size_t num_poses = 5;
  for (size_t i = 0; i < num_poses; i++) {
    pose_graph_tools::PoseGraphNode pg_node;
    pg_node.robot_id = 0;
    pg_node.key = i;
    pg_node.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(i, 0, 0)).matrix();
    pose_graph.nodes.push_back(pg_node);

    if (i == 0) {
      continue;
    }

    pose_graph_tools::PoseGraphEdge pg_edge;
    pg_edge.robot_from = 0;
    pg_edge.robot_to = 0;
    pg_edge.key_from = i - 1;
    pg_edge.key_to = i;
    pg_edge.pose = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)).matrix();
    pg_edge.type = pose_graph_tools::PoseGraphEdge::Type::ODOM;
    pose_graph.edges.push_back(pg_edge);
  }

  // Create variance mapping
  std::map<pose_graph_tools::PoseGraphEdge::Type, double> variance_map;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::ODOM] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::LOOPCLOSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::PRIOR] = 0.01;

  // Process pose graph
  graph.processPoseGraph(pose_graph, variance_map);

  auto values = graph.getValues();
  auto factors = graph.getFactors();
  EXPECT_EQ(num_poses - 1, factors->size());
  EXPECT_EQ(num_poses, values->size());
}

TEST(TestDeformationGraph, processMeshGraph) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  // Construct mesh graph
  auto mesh_graph = graph.getPoseGraph(true, false);

  // Create variance mapping
  std::map<pose_graph_tools::PoseGraphEdge::Type, double> variance_map;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::MESH_POSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::POSE_MESH] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::MESH] = 0.01;

  // Process pose graph
  DeformationGraph new_graph;
  new_graph.processMeshGraph(*mesh_graph, variance_map);

  auto values = graph.getValues();
  auto factors = graph.getFactors();
  auto new_values = new_graph.getValues();
  auto new_factors = new_graph.getFactors();
  EXPECT_EQ(new_values->size(), values->size());
  EXPECT_EQ(new_factors->size(), factors->size());
  for (const auto& key_value : *values) {
    EXPECT_EQ(values->at<gtsam::Pose3>(key_value.key).matrix(),
              new_values->at<gtsam::Pose3>(key_value.key).matrix());
  }
}

TEST(TestDeformationGraph, processPoseMeshGraph) {
  DeformationGraph graph;
  SetUpDeformationGraph(&graph);

  const auto original = makePartialCube();

  std::vector<uint64_t> new_node_valences{0, 2};
  graph.addNewNode(
      "a0"_id, 0, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 2, 2)), true);
  graph.processNodeValence("a0"_id, new_node_valences, 'v');
  graph.processNewBetween(
      "a0"_id, "a1"_id, gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 2)));

  // Construct mesh graph
  auto pose_graph = graph.getPoseGraph(false, true);
  auto mesh_graph = graph.getPoseGraph(true, false);

  // Create variance mapping
  std::map<pose_graph_tools::PoseGraphEdge::Type, double> variance_map;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::LOOPCLOSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::REJECTED_LOOPCLOSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::ODOM] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::PRIOR] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::MESH_POSE] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::POSE_MESH] = 0.01;
  variance_map[pose_graph_tools::PoseGraphEdge::Type::MESH] = 0.01;

  // Process pose graph
  DeformationGraph new_graph;
  new_graph.processPoseGraph(*pose_graph, variance_map);
  new_graph.processMeshGraph(*mesh_graph, variance_map);

  auto values = graph.getValues();
  auto factors = graph.getFactors();
  auto new_values = new_graph.getValues();
  auto new_factors = new_graph.getFactors();
  EXPECT_EQ(new_values->size(), values->size());
  EXPECT_EQ(new_factors->size(), factors->size());
  for (const auto& key_value : *values) {
    EXPECT_EQ(values->at<gtsam::Pose3>(key_value.key).matrix(),
              new_values->at<gtsam::Pose3>(key_value.key).matrix());
  }
}

}  // namespace kimera_pgmo
