#include <gtest/gtest.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/slam/BetweenFactor.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include <nlohmann/json.hpp>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"

namespace kimera_pgmo {
namespace {

using gtsam::Pose3;
using gtsam::Pose4DoF;
using gtsam::Symbol;

class CheckpointTest : public ::testing::Test {
 protected:
  void SetUp() override {
    path = std::filesystem::temp_directory_path() /
           ("pgmo-checkpoint-" + std::to_string(getpid()) + ".json");
  }

  void TearDown() override { std::filesystem::remove(path); }

  std::filesystem::path path;
};

void populate(DeformationGraph& graph) {
  const Pose3 first(gtsam::Rot3::Ypr(0.4, 0.2, -0.1), {0, 0, 0});
  const Pose3 second(gtsam::Rot3::Ypr(0.6, -0.1, 0.3), {1.2, 0.2, 0.1});
  graph.processNewNode(Symbol('a', 0), first, true);
  graph.processNewNode(Symbol('a', 1), second, false);
  graph.setPoseTimestamp(Symbol('a', 0), 1234567890123456789ULL);
  graph.setPoseTimestamp(Symbol('a', 1), 1234567891123456789ULL);
  graph.processNewBetween(Symbol('a', 0), Symbol('a', 1), first.between(second));

  gtsam::Values controls;
  controls.insert(Symbol('s', 0), Pose3(gtsam::Rot3(), {0, 1, 0}));
  controls.insert(Symbol('s', 1), Pose3(gtsam::Rot3(), {1, 1, 0}));
  std::vector<size_t> added;
  std::vector<Timestamp> stamps;
  graph.processNewMeshEdgesAndNodes({{Symbol('s', 0), Symbol('s', 1)}},
                                    controls,
                                    {{Symbol('s', 0), 1234567890123456789ULL},
                                     {Symbol('s', 1), 1234567891123456789ULL}},
                                    &added,
                                    &stamps,
                                    std::vector<double>{0.02});
  graph.processNodeValence(Symbol('a', 0), {0, 1}, 's');
  graph.processNodeValence(Symbol('a', 1), {0, 1}, 's');
  graph.processNewTempNode(Symbol('p', 0), second, true);
  graph.processNewTempBetween(Symbol('a', 1), Symbol('p', 0), Pose3());
  graph.processNodeValence(Symbol('p', 0), {0, 1}, 's', 0.01, true);
  gtsam::Matrix3 information = gtsam::Matrix3::Identity();
  information(0, 1) = information(1, 0) = 0.2;
  graph.processPointMeasurement(Symbol('p', 0),
                                Symbol('a', 0),
                                second,
                                first.translation(),
                                gtsam::noiseModel::Gaussian::Information(information),
                                true,
                                true);
}

void optimize(DeformationGraph& graph) {
  auto snapshot = graph.optimizationSnapshot();
  auto factors = snapshot.permanent.factors;
  factors.add(snapshot.temporary.factors);
  auto initial = snapshot.permanent.values;
  initial.insert(snapshot.temporary.values);
  const auto result = gtsam::LevenbergMarquardtOptimizer(factors, initial).optimize();
  gtsam::Values permanent, temporary;
  for (const auto& entry : result) {
    (snapshot.permanent.values.exists(entry.key) ? permanent : temporary)
        .insert(entry.key, entry.value);
  }

  graph.updateValues(permanent);
  graph.updateTempValues(temporary);
}

TEST_F(CheckpointTest, ResumeBothModesWithOriginalGeometryAndTemporaryConstraints) {
  for (const auto mode : {PoseMode::POSE3, PoseMode::POSE4DOF}) {
    DeformationGraph uninterrupted(false, mode);
    populate(uninterrupted);
    const auto original = uninterrupted.getInitialPose('a', 1);
    gtsam::Values shifted;
    shifted.insert(Symbol('a', 1),
                   Pose3(gtsam::Rot3::Ypr(0.8, 0.9, -0.8), {1.5, 0.4, 0.2}));
    uninterrupted.updateValues(shifted);
    uninterrupted.save(path);
    DeformationGraph resumed;
    resumed.load(path);
    EXPECT_EQ(resumed.poseMode(), mode);
    EXPECT_TRUE(resumed.getInitialPose('a', 1).equals(original));
    EXPECT_EQ(resumed.getPoseTimestamps().at(0).at(0), 1234567890123456789ULL);
    ASSERT_GT(resumed.getFactors()->size(), 0u);
    ASSERT_GT(resumed.getTempFactors()->size(), 0u);
    EXPECT_TRUE(resumed.getValues()->equals(*uninterrupted.getValues()));
    const auto noisy = dynamic_cast<const gtsam::NoiseModelFactor*>(
        resumed.getTempFactors()->back().get());
    ASSERT_NE(noisy, nullptr);
    const auto gaussian =
        dynamic_cast<const gtsam::noiseModel::Gaussian*>(noisy->noiseModel().get());
    ASSERT_NE(gaussian, nullptr);
    EXPECT_NEAR(gaussian->information()(0, 1), 0.2, 1e-10);
    for (const auto graph : {&uninterrupted, &resumed}) {
      const Pose3 delta(gtsam::Rot3::Yaw(0.1), {0.6, 0.1, 0.0});
      graph->updatePoseGraphInitialGuess(Symbol('a', 1), Symbol('a', 2), delta);
      graph->setPoseTimestamp(Symbol('a', 2), 1234567892123456789ULL);
      graph->processNewBetween(Symbol('a', 1), Symbol('a', 2), delta);
      graph->processNodeValence(Symbol('a', 2), {1}, 's');
      optimize(*graph);
    }

    EXPECT_TRUE(resumed.getValues()->equals(*uninterrupted.getValues(), 1e-7));
    EXPECT_TRUE(resumed.getTempValues()->equals(*uninterrupted.getTempValues(), 1e-7));
    if (mode == PoseMode::POSE4DOF) {
      const auto tilt = resumed.getValues()->at<Pose3>(Symbol('a', 1)).rotation().ypr();
      EXPECT_NEAR(tilt(1), -0.1, 1e-9);
      EXPECT_NEAR(tilt(2), 0.3, 1e-9);
    }

    const std::vector<Timestamp> stamps{1234567890123456789ULL};
    pcl::PointCloud<pcl::PointXYZRGBA> colored, out_a, out_b;
    pcl::PointXYZRGBA point;
    point.x = 0.5f;
    point.y = 1.0f;
    point.z = 0.0f;
    colored.push_back(point);
    out_a = colored;
    out_b = colored;
    uninterrupted.deformPoints(
        out_a, colored, stamps, 's', *uninterrupted.getValues(), 2, 10.0);
    resumed.deformPoints(out_b, colored, stamps, 's', *resumed.getValues(), 2, 10.0);
    ASSERT_EQ(out_b.size(), 1u);
    EXPECT_TRUE(out_a[0].getVector3fMap().isApprox(out_b[0].getVector3fMap(), 1e-6));
  }
}

TEST_F(CheckpointTest, FilteringRemappingAndRebuildPreserveAlignedState) {
  DeformationGraph graph(false, PoseMode::POSE4DOF);
  populate(graph);
  graph.updateInlierWeights(std::vector<double>(graph.getFactors()->size(), 0.75));
  graph.updateTempInlierWeights(
      std::vector<double>(graph.getTempFactors()->size(), 0.5));
  graph.save(path);
  DeformationGraph loaded;
  GraphLoadOptions options;
  options.include_priors = false;
  options.robot_id_remapping = {{0, 1}};
  loaded.load(path, options);
  EXPECT_EQ(loaded.getFactors()->size() + 1, graph.getFactors()->size());
  EXPECT_EQ(loaded.getInlierWeights()->size(), loaded.getFactors()->size());
  EXPECT_EQ(loaded.getPoseTimestamps().at(1).at(0), 1234567890123456789ULL);
  EXPECT_TRUE(loaded.getValues()->exists(Symbol('b', 0)));
  EXPECT_THROW(loaded.reindexMeshNodes('t', {{0, 0}}), std::invalid_argument);
  const Pose3 control(gtsam::Rot3::Yaw(0.3), {2, 3, 4});
  gtsam::Values updated;
  updated.insert(Symbol('t', 1), control);
  loaded.updateValues(updated);
  loaded.clearMeshEdgeFactorsOnly();
  loaded.reindexMeshNodes('t', {{1, 0}});
  EXPECT_TRUE(loaded.getValues()->at<Pose3>(Symbol('t', 0)).equals(control));
  EXPECT_EQ(loaded.getNumVertices(), 1u);
  loaded.save(path);
  DeformationGraph reloaded;
  reloaded.load(path);
  EXPECT_TRUE(reloaded.getValues()->at<Pose3>(Symbol('t', 0)).equals(control));
  reloaded.reindexMeshNodes('t', {});
  EXPECT_EQ(reloaded.getNumVertices(), 0u);
  EXPECT_FALSE(reloaded.getValues()->exists(Symbol('t', 0)));
  EXPECT_FALSE(reloaded.hasVertexKey('t'));
  reloaded.save(path);
  DeformationGraph empty_controls;
  empty_controls.load(path);
  EXPECT_FALSE(empty_controls.hasVertexKey('t'));
  EXPECT_EQ(empty_controls.getNumVertices(), 0u);
}

TEST_F(CheckpointTest, BadLoadLeavesLiveGraphIntact) {
  DeformationGraph graph(false, PoseMode::POSE4DOF);
  populate(graph);
  graph.save(path);
  const auto before = graph.getValuesCopy();
  nlohmann::json root;
  std::ifstream(path) >> root;
  root["permanent"]["factors"][0]["keys"][0] = "1";
  std::ofstream(path) << root;
  EXPECT_THROW(graph.load(path), std::invalid_argument);
  EXPECT_TRUE(graph.getValues()->equals(before));
  root["version"] = 100;
  std::ofstream(path) << root;
  EXPECT_THROW(graph.load(path), std::invalid_argument);
  EXPECT_TRUE(graph.getValues()->equals(before));
}

TEST_F(CheckpointTest, NativeGncRejectsConflictingLoopAndPreservesWeights) {
  for (const auto mode : {PoseMode::POSE3, PoseMode::POSE4DOF}) {
    DeformationGraph graph(false, mode);
    pose_graph_tools::PoseGraph input;
    for (size_t i = 0; i < 4; ++i) {
      pose_graph_tools::PoseGraphNode node;
      node.robot_id = 0;
      node.key = i;
      node.stamp_ns = 100 + i;
      node.pose = Eigen::Isometry3d::Identity();
      node.pose.translation().x() = static_cast<double>(i);
      input.nodes.push_back(node);
    }

    for (size_t i = 1; i < 4; ++i) {
      pose_graph_tools::PoseGraphEdge edge;
      edge.robot_from = edge.robot_to = 0;
      edge.key_from = i - 1;
      edge.key_to = i;
      edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
      edge.pose = Eigen::Isometry3d::Identity();
      edge.pose.translation().x() = 1.0;
      input.edges.push_back(edge);
    }

    graph.processPoseGraph(input, {{pose_graph_tools::PoseGraphEdge::ODOM, 0.01}});
    graph.processNodeMeasurements({{Symbol('a', 0), Pose3()}}, 1e-8);
    graph.processNewBetween(
        Symbol('a', 0), Symbol('a', 3), Pose3(gtsam::Rot3(), {100, 0, 0}), 0.01);
    const auto snapshot = graph.optimizationSnapshot();
    KimeraRpgoOptimizer::Config config;
    config.use_gnc = true;
    config.print_summary = false;
    config.print_iterations = false;
    KimeraRpgoOptimizer optimizer(config);
    optimizer.update(snapshot.permanent.factors,
                     snapshot.permanent.values,
                     snapshot.permanent.known_inliers,
                     {},
                     {},
                     {});
    EXPECT_NEAR(optimizer.getEstimates().at<Pose3>(Symbol('a', 3)).x(), 3.0, 0.01);
    ASSERT_EQ(optimizer.getInlierWeights().size(), snapshot.permanent.factors.size());
    EXPECT_LT(optimizer.getInlierWeights().back(), 0.5);
    graph.updateValues(optimizer.getEstimates());
    graph.updateInlierWeights(optimizer.getInlierWeights());
    graph.save(path);
    DeformationGraph loaded;
    loaded.load(path);
    EXPECT_EQ(*graph.getInlierWeights(), *loaded.getInlierWeights());
    const auto exported = loaded.getPoseGraph(loaded.getPoseTimestamps(), false, true);
    ASSERT_EQ(exported->nodes.size(), 4u);
    ASSERT_EQ(exported->edges.size(), 5u);
    EXPECT_EQ(exported->edges.back().type,
              pose_graph_tools::PoseGraphEdge::REJECTED_LOOPCLOSE);
  }
}

TEST_F(CheckpointTest, RobotCollisionAndPriorFilteringAreTransactional) {
  DeformationGraph graph;
  graph.processNewNode(Symbol('a', 0), Pose3(), true);
  graph.processNewNode(Symbol('b', 0), Pose3(gtsam::Rot3(), {2, 0, 0}), true);
  graph.processNewBetween(
      Symbol('a', 0), Symbol('b', 0), Pose3(gtsam::Rot3(), {2, 0, 0}));
  graph.save(path);
  const auto before = graph.getValuesCopy();
  EXPECT_THROW(graph.load(path, true, true, 0), std::invalid_argument);
  GraphLoadOptions options;
  options.robot_id_remapping = {{1, 0}};
  EXPECT_THROW(graph.load(path, options), std::invalid_argument);
  EXPECT_TRUE(graph.getValues()->equals(before));
  options.robot_id_remapping = {{0, 1}, {1, 0}};
  graph.load(path, options);
  EXPECT_NEAR(graph.getValues()->at<Pose3>(Symbol('a', 0)).x(), 2.0, 1e-10);
  graph.clear();
  EXPECT_EQ(graph.getNumLoopclosures(), 0u);
  EXPECT_TRUE(graph.getKnownInlierSet()->empty());
  EXPECT_TRUE(graph.getTempKnownInlierSet()->empty());
}

TEST_F(CheckpointTest, FailedSavePreservesPreviousCheckpoint) {
  DeformationGraph graph;
  populate(graph);
  graph.save(path);
  const auto noise = gtsam::noiseModel::Robust::Create(
      gtsam::noiseModel::mEstimator::Huber::Create(1.0),
      gtsam::noiseModel::Isotropic::Variance(3, 1.0));
  graph.processPointMeasurement(
      Symbol('a', 0), Symbol('a', 1), Pose3(), {1, 0, 0}, noise);
  EXPECT_THROW(graph.save(path), std::invalid_argument);
  DeformationGraph previous;
  previous.load(path);
  EXPECT_EQ(previous.getFactors()->size() + 1, graph.getFactors()->size());
  GraphLoadOptions options;
  options.include_temp = false;
  options.include_priors = false;
  previous.load(path, options);
  EXPECT_TRUE(previous.getTempValues()->empty());
  EXPECT_TRUE(previous.getTempFactors()->empty());
  EXPECT_EQ(previous.getFactors()->size() + 2, graph.getFactors()->size());
}

}  // namespace
}  // namespace kimera_pgmo
