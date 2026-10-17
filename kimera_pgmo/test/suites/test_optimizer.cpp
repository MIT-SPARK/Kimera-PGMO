#include <gtest/gtest.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/deformation_graph_4dof.h"
#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"

namespace kimera_pgmo {
namespace {

class OptimizerExtrasTest : public ::testing::TestWithParam<bool> {};

TEST_P(OptimizerExtrasTest, PartitionsResultsAndDoesNotRetainExtras) {
  KimeraRpgoOptimizer::Config config;
  config.print_iterations = false;
  KimeraRpgoOptimizer optimizer(config);
  const auto base_key = gtsam::Symbol('a', 0);
  const auto extra_key = gtsam::Symbol('p', 0);
  const auto origin = gtsam::Pose3();
  const auto displacement = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 0));
  const auto noise = gtsam::noiseModel::Isotropic::Sigma(6, 0.1);
  DeformationGraph graph;
  graph.addNewNode(base_key, 0, origin, true);
  auto base = graph.getOptimizationProblem(GetParam());
  base.inliers.insert(0);
  OptimizationProblem extras;
  if (GetParam()) {
    extras.initial.insert(extra_key, gtsam::Pose4DoF(origin));
    extras.factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose4DoF>>(
        base_key,
        extra_key,
        gtsam::Pose4DoF(displacement),
        gtsam::noiseModel::Isotropic::Sigma(4, 0.1));
  } else {
    extras.initial.insert(extra_key, origin);
    extras.factors.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(
        base_key, extra_key, displacement, noise);
  }

  extras.inliers.insert(0);

  const auto result = optimizer.update(base, extras);
  ASSERT_EQ(result.estimates.size(), 1u);
  ASSERT_EQ(result.extra_estimates.size(), 1u);
  EXPECT_TRUE(convertPose4DoFToPose3(result.estimates)
                  .at<gtsam::Pose3>(base_key)
                  .equals(origin, 1e-5));
  EXPECT_TRUE(convertPose4DoFToPose3(result.extra_estimates)
                  .at<gtsam::Pose3>(extra_key)
                  .equals(displacement, 1e-5));
  ASSERT_EQ(result.weights.size(), 1u);
  ASSERT_EQ(result.extra_weights.size(), 1u);
  EXPECT_DOUBLE_EQ(result.weights.front(), 1.0);
  EXPECT_DOUBLE_EQ(result.extra_weights.front(), 1.0);
  EXPECT_TRUE(convertPose4DoFToPose3(extras.initial)
                  .at<gtsam::Pose3>(extra_key)
                  .equals(origin));

  EXPECT_EQ(result.estimates.at(base_key).dim(), GetParam() ? 4u : 6u);
  EXPECT_EQ(result.extra_estimates.at(extra_key).dim(), GetParam() ? 4u : 6u);
  graph.updateOptimizationResult(result);
  EXPECT_TRUE(graph.getValues()->at<gtsam::Pose3>(base_key).equals(origin, 1e-5));

  const auto base_only = optimizer.update(base);
  ASSERT_EQ(base_only.estimates.size(), 1u);
  EXPECT_TRUE(convertPose4DoFToPose3(base_only.estimates)
                  .at<gtsam::Pose3>(base_key)
                  .equals(origin, 1e-5));
  EXPECT_TRUE(base_only.extra_estimates.empty());
  EXPECT_TRUE(base_only.extra_weights.empty());
}

INSTANTIATE_TEST_SUITE_P(PoseRepresentations, OptimizerExtrasTest, ::testing::Bool());

TEST(OptimizerExtras, ExtraInlierIndicesAreLocal) {
  KimeraRpgoOptimizer::Config config;
  config.print_iterations = false;
  KimeraRpgoOptimizer optimizer(config);
  OptimizationProblem base;
  const auto key = gtsam::Symbol('a', 0);
  base.initial.insert(key, gtsam::Pose3());
  base.factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      key, gtsam::Pose3(), gtsam::noiseModel::Isotropic::Sigma(6, 0.01));
  base.inliers.insert(0);
  OptimizationProblem extras;
  extras.factors.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(
      key,
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(50, 0, 0)),
      gtsam::noiseModel::Isotropic::Sigma(6, 1.0));
  extras.inliers.insert(0);

  const auto result = optimizer.update(base, extras);
  ASSERT_EQ(result.extra_weights.size(), 1u);
  EXPECT_DOUBLE_EQ(result.extra_weights.front(), 1.0);
  EXPECT_TRUE(result.extra_estimates.empty());
  EXPECT_NEAR(result.estimates.at<gtsam::Pose3>(key).x(), 50.0 / 10001.0, 1e-5);
}

TEST(OptimizerExtras, RejectsAmbiguousKeysAndInvalidInliers) {
  KimeraRpgoOptimizer optimizer(KimeraRpgoOptimizer::Config{});
  OptimizationProblem base;
  base.initial.insert(0, gtsam::Pose3());
  OptimizationProblem extras = base;
  EXPECT_THROW(optimizer.update(base, extras), std::invalid_argument);
  extras.initial.clear();
  extras.inliers.insert(0);
  EXPECT_THROW(optimizer.update(base, extras), std::invalid_argument);
  extras.inliers.clear();
  base.inliers.insert(0);
  EXPECT_THROW(optimizer.update(base, extras), std::invalid_argument);
}

TEST(OptimizerExtras, FilteringFactorsPreservesInlierAndWeightIndices) {
  DeformationGraph graph;
  const auto pose_key = gtsam::Symbol('a', 0);
  graph.addNewNode(pose_key, 0, gtsam::Pose3(), true);
  graph.processPointMeasurement(pose_key,
                                gtsam::Symbol('v', 0),
                                gtsam::Pose3(),
                                gtsam::Point3(1, 0, 0),
                                0.1,
                                true);
  graph.updateInlierWeights({0.25, 0.75});
  const auto path =
      std::filesystem::temp_directory_path() / "pgmo-extras-metadata.json";
  graph.save(path.string());
  const auto loaded = DeformationGraph::load(path, false);
  std::filesystem::remove(path);
  ASSERT_TRUE(loaded);
  EXPECT_EQ(loaded->getOptimizationProblem().inliers, std::set<size_t>({0}));
  EXPECT_EQ(*loaded->getInlierWeights(), std::vector<double>({0.75}));

  graph.removePriorsWithPrefix('a');
  EXPECT_EQ(graph.getOptimizationProblem().inliers, std::set<size_t>({0}));
  EXPECT_EQ(*graph.getInlierWeights(), std::vector<double>({0.75}));
  graph.clearMeshNodesOnly();
  EXPECT_TRUE(graph.getOptimizationProblem().factors.empty());
  EXPECT_TRUE(graph.getOptimizationProblem().inliers.empty());
  EXPECT_TRUE(graph.getInlierWeights()->empty());

  graph.processPointMeasurement(pose_key,
                                gtsam::Symbol('v', 0),
                                gtsam::Pose3(),
                                gtsam::Point3(1, 0, 0),
                                0.1,
                                true);
  graph.updateInlierWeights({1.0});
  graph.clearFactors();
  EXPECT_TRUE(graph.getOptimizationProblem().inliers.empty());
  EXPECT_TRUE(graph.getInlierWeights()->empty());
}

TEST(OptimizerExtras, GraphAppliesOnlyBaseResults) {
  DeformationGraph graph;
  const auto key = gtsam::Symbol('a', 0);
  graph.addNewNode(key, 0, gtsam::Pose3(), true);
  auto problem = graph.getOptimizationProblem();
  ASSERT_EQ(problem.initial.size(), 1u);
  ASSERT_EQ(problem.factors.size(), 1u);
  OptimizationResult result;
  const auto expected = gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 2, 3));
  result.estimates.insert(key, expected);
  result.extra_estimates.insert(gtsam::Symbol('p', 0), expected);
  result.weights = {0.75};
  graph.updateOptimizationResult(result);
  ASSERT_EQ(graph.getValues()->size(), 1u);
  EXPECT_TRUE(graph.getValues()->at<gtsam::Pose3>(key).equals(expected));
  EXPECT_EQ(*graph.getInlierWeights(), result.weights);
  EXPECT_TRUE(problem.initial.at<gtsam::Pose3>(key).equals(gtsam::Pose3()));
}

}  // namespace
}  // namespace kimera_pgmo
