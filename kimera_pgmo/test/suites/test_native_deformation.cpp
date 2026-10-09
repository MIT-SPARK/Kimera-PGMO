#include <gtest/gtest.h>
#include <gtsam/base/numericalDerivative.h>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"

namespace kimera_pgmo {
namespace {

using gtsam::Pose3;
using gtsam::Pose4DoF;
using gtsam::Symbol;

template <typename From, typename To>
void checkDerivative(const From& from, const To& to) {
  const DeformationEdgeFactorT<From, To> factor(
      0,
      1,
      gtsam::Point3(0.3, -1.1, 2.0),
      gtsam::noiseModel::Isotropic::Variance(3, 0.1));
  gtsam::Matrix H1, H2;
  factor.evaluateError(from, to, H1, H2);
  const std::function<gtsam::Vector3(const From&, const To&)> error =
      [&](const From& a, const To& b) { return factor.evaluateError(a, b); };
  EXPECT_TRUE(H1.isApprox(
      (gtsam::numericalDerivative21<gtsam::Vector3, From, To>(error, from, to)), 1e-6));
  EXPECT_TRUE(H2.isApprox(
      (gtsam::numericalDerivative22<gtsam::Vector3, From, To>(error, from, to)), 1e-6));
}

TEST(NativeDeformation, TiltedEndpointJacobians) {
  const Pose3 from(gtsam::Rot3::Ypr(1.0, 0.4, -0.3), {0.3, 0.7, 1.0});
  const Pose3 to(gtsam::Rot3::Ypr(-0.7, -0.2, 0.5), {1.2, -0.6, 0.1});
  checkDerivative(from, to);
  checkDerivative(Pose4DoF(from), to);
  checkDerivative(from, Pose4DoF(to));
  checkDerivative(Pose4DoF(from), Pose4DoF(to));
}

TEST(NativeDeformation, RpgoSupportsNativeValuesAndRejectsPcm) {
  DeformationGraph graph(false, PoseMode::POSE4DOF);
  const Pose3 first(gtsam::Rot3::Ypr(0.4, 0.2, -0.1), {0, 0, 0});
  const Pose3 second(gtsam::Rot3::Ypr(0.6, -0.1, 0.3), {1.2, 0.2, 0.1});
  graph.processNewNode(Symbol('a', 0), first, true);
  graph.processNewNode(Symbol('a', 1), second, false);
  graph.processNewBetween(Symbol('a', 0), Symbol('a', 1), first.between(second));
  graph.processNewTempNode(Symbol('p', 0), second, false);
  graph.processNewTempBetween(Symbol('a', 1), Symbol('p', 0), Pose3());
  auto input = graph.optimizationSnapshot();
  KimeraRpgoOptimizer::Config config;
  config.use_gnc = false;
  config.print_summary = false;
  config.print_iterations = false;
  KimeraRpgoOptimizer optimizer(config);
  optimizer.update(input.permanent.factors,
                   input.permanent.values,
                   input.permanent.known_inliers,
                   input.temporary.factors,
                   input.temporary.values,
                   input.temporary.known_inliers);
  EXPECT_EQ(optimizer.getEstimates().size(), input.permanent.values.size());
  EXPECT_TRUE(optimizer.getEstimates().at<Pose3>(Symbol('a', 0)).translation().norm() <
              1e-6);
  ASSERT_EQ(optimizer.getTempEstimates().size(), 1u);
  EXPECT_TRUE(
      optimizer.getTempEstimates().at<Pose3>(Symbol('p', 0)).equals(second, 1e-6));
  config.use_pcm = true;
  KimeraRpgoOptimizer pcm(config);
  EXPECT_THROW(pcm.update(input.permanent.factors,
                          input.permanent.values,
                          {},
                          input.temporary.factors,
                          input.temporary.values,
                          {}),
               std::invalid_argument);
}

}  // namespace
}  // namespace kimera_pgmo
