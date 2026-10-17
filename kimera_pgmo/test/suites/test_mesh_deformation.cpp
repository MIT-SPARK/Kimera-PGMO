/**
 * @file   test_mesh_defomration.cpp
 * @brief  Unit-tests for mesh deformation
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <gtsam/geometry/Pose3.h>

#include "kimera_pgmo/mesh_deformation.h"
#include "kimera_pgmo/pcl_mesh_traits.h"

namespace kimera_pgmo {

TEST(TestMeshDeformation, deformPoints) {
  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;

  PointCloud original;
  std::vector<StampedPoint> control_points;
  gtsam::Values values;
  char prefix = 'a';
  for (size_t i = 0; i < 100; i++) {
    original.push_back(Point(static_cast<double>(i), 0.0, 0.0));
    if (i % 10 == 0) {
      control_points.push_back({0, gtsam::Point3(static_cast<double>(i), 0.0, 0.0)});

      values.insert(
          gtsam::Symbol(prefix, static_cast<int>(i / 10)),
          gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(static_cast<double>(i), 1.0, 0.0)));
    }
  }

  deformation::deformPoints(original, original, prefix, control_points, values);

  ASSERT_EQ(100, original.size());
  for (size_t i = 0; i < 100; i++) {
    EXPECT_NEAR(static_cast<double>(i), original.points[i].x, 1.0e-9);
    EXPECT_NEAR(1.0, original.points[i].y, 1.0e-9);
    EXPECT_NEAR(0.0, original.points[i].z, 1.0e-9);
  }
}

TEST(TestMeshDeformation, deformPointsWithTimeCheck) {
  typedef pcl::PointXYZ Point;
  typedef pcl::PointCloud<Point> PointCloud;

  PointCloud original_points;
  std::vector<Timestamp> stamps;
  std::vector<StampedPoint> control_points;
  gtsam::Values optimized_values;
  char prefix = 'a';
  for (size_t i = 0; i < 100; i++) {
    original_points.push_back(Point(static_cast<double>(i), 0.0, 0.0));
    if (i % 10 == 0) {
      if (i > 50) {
        optimized_values.insert(
            gtsam::Symbol(prefix, static_cast<int>(i / 10)),
            gtsam::Pose3(gtsam::Rot3(),
                         gtsam::Point3(static_cast<double>(i), 1.0, 0.0)));
        control_points.push_back(
            {stampFromSec(20.0), Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0)});
      } else {
        optimized_values.insert(
            gtsam::Symbol(prefix, static_cast<int>(i / 10)),
            gtsam::Pose3(gtsam::Rot3(),
                         gtsam::Point3(static_cast<double>(i), -1.0, 0.0)));
        control_points.push_back(
            {0, Eigen::Vector3d(static_cast<double>(i), 0.0, 0.0)});
      }
    }

    if (i < 50) {
      stamps.push_back(0);
    } else {
      stamps.push_back(stampFromSec(20.0));
    }
  }

  const ConstStampedCloud<pcl::PointXYZ> cloud{original_points, stamps};
  deformation::deformPoints(
      original_points, cloud, prefix, control_points, optimized_values, 3, 10.0);

  ASSERT_EQ(100, original_points.size());
  for (size_t i = 0; i < 50; i++) {
    EXPECT_EQ(static_cast<double>(i), original_points.points[i].x);
    ASSERT_EQ(-1.0, original_points.points[i].y);
    ASSERT_EQ(0.0, original_points.points[i].z);
  }
  for (size_t i = 51; i < 100; i++) {
    ASSERT_EQ(static_cast<double>(i), original_points.points[i].x);
    ASSERT_EQ(1.0, original_points.points[i].y);
    ASSERT_EQ(0.0, original_points.points[i].z);
  }
}

}  // namespace kimera_pgmo
