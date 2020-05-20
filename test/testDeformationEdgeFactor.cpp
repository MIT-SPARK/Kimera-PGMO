#include "gtest/gtest.h"

#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include "mesher_mapper/DeformationGraph.h"
#include "test_config.h"

using gtsam::Point3;
using gtsam::Pose3;
using gtsam::Rot3;

namespace mesher_mapper {
TEST(DeformationEdgeFactor, ZeroRotation) {
  Point3 node_1 = Point3(0, 0, 0);
  Point3 node_2 = Point3(1, 1, 1);

  Pose3 pose_1 = Pose3();
  Pose3 pose_2 = Pose3(Rot3(), Point3(1, 0, 0));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);

  gtsam::Matrix actualH1, actualH2;
  gtsam::Vector actual =
      factor.evaluateError(pose_1, pose_2, actualH1, actualH2);
  gtsam::Vector expected = Point3(-1, 0, 0).vector();

  EXPECT_TRUE(gtsam::assert_equal(expected, actual));

  gtsam::Matrix numericalH1 =
      gtsam::numericalDerivative21<gtsam::Vector3, Pose3, Pose3>(
          boost::function<gtsam::Vector(const Pose3&, const Pose3&)>(
              boost::bind(&DeformationEdgeFactor::evaluateError,
                          factor,
                          _1,
                          _2,
                          boost::none,
                          boost::none)),
          pose_1,
          pose_2,
          1e-5);
  EXPECT_TRUE(gtsam::assert_equal(numericalH1, actualH1, 1E-5));
  gtsam::Matrix numericalH2 =
      gtsam::numericalDerivative22<gtsam::Vector3, Pose3, Pose3>(
          boost::function<gtsam::Vector(const Pose3&, const Pose3&)>(
              boost::bind(&DeformationEdgeFactor::evaluateError,
                          factor,
                          _1,
                          _2,
                          boost::none,
                          boost::none)),
          pose_1,
          pose_2,
          1e-5);
  EXPECT_TRUE(gtsam::assert_equal(numericalH2, actualH2, 1E-5));
}

TEST(DeformationEdgeFactor, ZeroTranslation1) {
  Point3 node_1 = Point3(0, 0, 0);
  Point3 node_2 = Point3(1, 1, 1);

  Pose3 pose_1 = Pose3();
  Pose3 pose_2 = Pose3(Rot3(0, 1, 0, 0), Point3());

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);

  gtsam::Matrix actualH1, actualH2;
  gtsam::Vector actual =
      factor.evaluateError(pose_1, pose_2, actualH1, actualH2);
  gtsam::Vector expected = Point3(0, 0, 0).vector();

  EXPECT_TRUE(gtsam::assert_equal(expected, actual));

  gtsam::Matrix numericalH1 =
      gtsam::numericalDerivative21<gtsam::Vector3, Pose3, Pose3>(
          boost::function<gtsam::Vector(const Pose3&, const Pose3&)>(
              boost::bind(&DeformationEdgeFactor::evaluateError,
                          factor,
                          _1,
                          _2,
                          boost::none,
                          boost::none)),
          pose_1,
          pose_2,
          1e-5);
  EXPECT_TRUE(gtsam::assert_equal(numericalH1, actualH1, 1E-5));
  gtsam::Matrix numericalH2 =
      gtsam::numericalDerivative22<gtsam::Vector3, Pose3, Pose3>(
          boost::function<gtsam::Vector(const Pose3&, const Pose3&)>(
              boost::bind(&DeformationEdgeFactor::evaluateError,
                          factor,
                          _1,
                          _2,
                          boost::none,
                          boost::none)),
          pose_1,
          pose_2,
          1e-5);
  EXPECT_TRUE(gtsam::assert_equal(numericalH2, actualH2, 1E-5));
}

TEST(DeformationEdgeFactor, ZeroTranslation2) {
  Point3 node_1 = Point3(0, 0, 0);
  Point3 node_2 = Point3(1, 1, 1);

  Pose3 pose_1 = Pose3(Rot3(0, 0, 0, 1), Point3());
  Pose3 pose_2 = Pose3();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);

  gtsam::Matrix actualH1, actualH2;
  gtsam::Vector actual =
      factor.evaluateError(pose_1, pose_2, actualH1, actualH2);
  gtsam::Vector expected = Point3(-2, -2, 0).vector();

  EXPECT_TRUE(gtsam::assert_equal(expected, actual));

  gtsam::Matrix numericalH1 =
      gtsam::numericalDerivative21<gtsam::Vector3, Pose3, Pose3>(
          boost::function<gtsam::Vector(const Pose3&, const Pose3&)>(
              boost::bind(&DeformationEdgeFactor::evaluateError,
                          factor,
                          _1,
                          _2,
                          boost::none,
                          boost::none)),
          pose_1,
          pose_2,
          1e-5);
  EXPECT_TRUE(gtsam::assert_equal(numericalH1, actualH1, 1E-5));
  gtsam::Matrix numericalH2 =
      gtsam::numericalDerivative22<gtsam::Vector3, Pose3, Pose3>(
          boost::function<gtsam::Vector(const Pose3&, const Pose3&)>(
              boost::bind(&DeformationEdgeFactor::evaluateError,
                          factor,
                          _1,
                          _2,
                          boost::none,
                          boost::none)),
          pose_1,
          pose_2,
          1e-5);
  EXPECT_TRUE(gtsam::assert_equal(numericalH2, actualH2, 1E-5));
}

TEST(DeformationEdgeFactor, Simple) {
  Point3 node_1 = Point3(0, 1, 0);
  Point3 node_2 = Point3(1, 3, 1);

  Pose3 pose_1 = Pose3(Rot3::Rodrigues(0.1, 0.2, 0.3), Point3(0.1, 0.1, -0.2));
  Pose3 pose_2 = Pose3(Rot3::Rodrigues(0.3, 0.4, 0.5), Point3(0.2, 0.1, 0.2));

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(3, 1e-3);

  DeformationEdgeFactor factor(1, 2, node_1, node_2, noise);

  gtsam::Matrix actualH1, actualH2;
  gtsam::Vector actual =
      factor.evaluateError(pose_1, pose_2, actualH1, actualH2);
  gtsam::Vector expected = node_1 + pose_1.transform_from(node_2 - node_1) -
                           (node_2 + pose_2.translation());

  EXPECT_TRUE(gtsam::assert_equal(expected, actual));

  gtsam::Matrix numericalH1 =
      gtsam::numericalDerivative21<gtsam::Vector3, Pose3, Pose3>(
          boost::function<gtsam::Vector(const Pose3&, const Pose3&)>(
              boost::bind(&DeformationEdgeFactor::evaluateError,
                          factor,
                          _1,
                          _2,
                          boost::none,
                          boost::none)),
          pose_1,
          pose_2,
          1e-5);
  EXPECT_TRUE(gtsam::assert_equal(numericalH1, actualH1, 1E-5));
  gtsam::Matrix numericalH2 =
      gtsam::numericalDerivative22<gtsam::Vector3, Pose3, Pose3>(
          boost::function<gtsam::Vector(const Pose3&, const Pose3&)>(
              boost::bind(&DeformationEdgeFactor::evaluateError,
                          factor,
                          _1,
                          _2,
                          boost::none,
                          boost::none)),
          pose_1,
          pose_2,
          1e-5);
  EXPECT_TRUE(gtsam::assert_equal(numericalH2, actualH2, 1E-5));
}
}  // namespace mesher_mapper