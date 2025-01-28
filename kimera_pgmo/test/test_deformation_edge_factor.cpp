#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "gtest/gtest.h"
#include "kimera_pgmo/deformation_graph.h"
#include "test_config.h"

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

namespace kimera_pgmo {

template <typename Factor, typename Error, typename V1, typename V2>
void evaluateFactor(const Factor& factor,
                    const V1& v1,
                    const V2& v2,
                    const Error& expected,
                    double tol,
                    double delta = 1.0e-5) {
  gtsam::Matrix H1_actual, H2_actual;
#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
  const auto actual = factor.evaluateError(v1, v2, H1_actual, H2_actual);
#else
  const auto actual = factor.evaluateError(v1, v2, &H1_actual, &H2_actual);
#endif

  const auto H1_expected = gtsam::numericalDerivative21<gtsam::Vector, V1, V2>(
      [&](const auto& v1, const auto& v2) { return factor.evaluateError(v1, v2); },
      v1,
      v2,
      delta);

  const auto H2_expected = gtsam::numericalDerivative22<gtsam::Vector, V1, V2>(
      [&](const auto& v1, const auto& v2) { return factor.evaluateError(v1, v2); },
      v1,
      v2,
      delta);

  // Verify the Jacobians are correct
  EXPECT_TRUE(gtsam::assert_equal(expected, actual, tol));
  EXPECT_TRUE(gtsam::assert_equal(H1_expected, H1_actual, tol));
  EXPECT_TRUE(gtsam::assert_equal(H2_expected, H2_actual, tol));
}

TEST(TestDeformationEdgeFactor, zeroRotation) {
  Pose3 node_1 = Pose3(Rot3(), Point3(0, 0, 0));
  Point3 node_2 = Point3(1, 1, 1);

  Pose3 pose_1 = Pose3();
  Pose3 pose_2 = Pose3(Rot3(), Point3(2, 1, 1));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  const gtsam::Vector expected = Point3(-1, 0, 0);
  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);
  evaluateFactor(factor, pose_1, pose_2, expected, 1.0e-5);
}

TEST(TestDeformationEdgeFactor, zeroTranslation1) {
  Pose3 node_1 = Pose3(Rot3(), Point3(0, 0, 0));
  Point3 node_2 = Point3(1, 1, 1);

  Pose3 pose_1 = Pose3();
  Pose3 pose_2 = Pose3(Rot3(0, 1, 0, 0), node_2);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  gtsam::Vector expected = Point3(0, 0, 0);
  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);
  evaluateFactor(factor, pose_1, pose_2, expected, 1.0e-5);
}

TEST(TestDeformationEdgeFactor, zeroTranslation2) {
  Pose3 node_1 = Pose3(Rot3(), Point3(0, 0, 0));
  Point3 node_2 = Point3(1, 1, 1);

  Pose3 pose_1 = Pose3(Rot3(0, 0, 0, 1), Eigen::Vector3d::Zero());
  Pose3 pose_2 = Pose3(Rot3(), node_2);

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  gtsam::Vector expected = Point3(-2, -2, 0);
  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);
  evaluateFactor(factor, pose_1, pose_2, expected, 1.0e-5);
}

TEST(TestDeformationEdgeFactor, simple) {
  Pose3 node_1 = Pose3(Rot3(), Point3(0, 1, 0));
  Point3 node_2 = Point3(1, 3, 1);

  Pose3 pose_1 = Pose3(Rot3::Rodrigues(0.1, 0.2, 0.3), Point3(0.1, 1.1, -0.2));
  Pose3 pose_2 = Pose3(Rot3::Rodrigues(0.3, 0.4, 0.5), Point3(1.2, 3.1, 1.2));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  gtsam::Vector expected =
      pose_1.transformFrom((node_2 - node_1.translation()).eval()) -
      pose_2.translation();
  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);
  evaluateFactor(factor, pose_1, pose_2, expected, 1.0e-5);
}

TEST(TestDeformationEdgeFactor, relative1) {
  Pose3 node_1 = Pose3(Rot3(0, 0, 0, 1), Point3(1, 0, 0));
  Point3 node_2 = Point3(1, 1, 1);

  Pose3 pose_1 = Pose3(Rot3(0, 0, 0, 1), Point3(1, 0, 0));
  Pose3 pose_2 = Pose3(Rot3(), Point3(1, 1, 1));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  gtsam::Vector expected = Point3(0, 0, 0);
  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);
  evaluateFactor(factor, pose_1, pose_2, expected, 1.0e-5);
}

TEST(TestDeformationEdgeFactor, relative2) {
  Pose3 node_1 = Pose3(Rot3(0, 0, 0, 1), Point3(1, 0, 0));
  Point3 node_2 = Point3(1, 1, 1);

  Pose3 pose_1 = Pose3(Rot3(), Point3(1, 0, 0));
  Pose3 pose_2 = Pose3(Rot3(), Point3(1, 1, 1));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  gtsam::Vector expected = Point3(0, -2, 0);
  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);
  evaluateFactor(factor, pose_1, pose_2, expected, 1.0e-5);
}

}  // namespace kimera_pgmo
