/**
 * @file   test_gtsam_functions.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <kimera_pgmo/utils/common_structs.h>

#include "kimera_pgmo_ros/conversion/gtsam_conversions.h"

namespace kimera_pgmo {

using namespace conversions;

TEST(TestGtsamConversions, RosPoseToGtsam) {
  geometry_msgs::Pose ros_pose;
  gtsam::Pose3 gtsam_pose = RosToGtsam(ros_pose);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), gtsam_pose));

  ros_pose.position.x = 1;
  ros_pose.position.z = 100;
  ros_pose.orientation.y = 0.707;
  ros_pose.orientation.w = 0.707;
  gtsam_pose = RosToGtsam(ros_pose);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0.707, 0, 0.707, 0), gtsam::Point3(1, 0, 100)),
      gtsam_pose));
}

TEST(TestGtsamConversions, GtsamPoseToRos) {
  gtsam::Pose3 gtsam_pose;
  geometry_msgs::Pose ros_pose = GtsamToRos(gtsam_pose);
  EXPECT_EQ(0, ros_pose.position.x);
  EXPECT_EQ(0, ros_pose.position.y);
  EXPECT_EQ(0, ros_pose.position.z);
  EXPECT_EQ(0, ros_pose.orientation.x);
  EXPECT_EQ(0, ros_pose.orientation.y);
  EXPECT_EQ(0, ros_pose.orientation.z);
  EXPECT_EQ(1, ros_pose.orientation.w);

  gtsam_pose = gtsam::Pose3(gtsam::Rot3(0.707, 0, 0.707, 0), gtsam::Point3(1, 0, 100));
  ros_pose = GtsamToRos(gtsam_pose);
  EXPECT_EQ(1, ros_pose.position.x);
  EXPECT_EQ(0, ros_pose.position.y);
  EXPECT_EQ(100, ros_pose.position.z);
  EXPECT_NEAR(0, ros_pose.orientation.x, 0.001);
  EXPECT_NEAR(0.707, ros_pose.orientation.y, 0.001);
  EXPECT_NEAR(0, ros_pose.orientation.z, 0.001);
  EXPECT_NEAR(0.707, ros_pose.orientation.w, 0.001);
}

// GTSAM graph to ROS
TEST(TestGtsamConversions, GtsamGraphToRos) {
  ros::Time::init();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);
  gtsam::NonlinearFactorGraph nfg;
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise));
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1),
      gtsam::Symbol('a', 2),
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 1, 1)),
      noise));

  gtsam::Values val;
  std::map<size_t, std::vector<Timestamp> > time_stamps;
  time_stamps[0] = std::vector<Timestamp>{};
  val.insert(gtsam::Symbol('a', 0), gtsam::Pose3());
  time_stamps[0].push_back(stampFromSec(0.01));
  val.insert(gtsam::Symbol('a', 1), gtsam::Pose3());
  time_stamps[0].push_back(stampFromSec(0.02));
  val.insert(gtsam::Symbol('a', 2),
             gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 1, 1)));
  time_stamps[0].push_back(stampFromSec(0.03));

  const GraphMsgPtr& pose_graph_ptr = GtsamGraphToRos(nfg, val, time_stamps);

  // Check edges
  EXPECT_EQ(size_t(2), pose_graph_ptr->edges.size());
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(), RosToGtsam(pose_graph_ptr->edges[0].pose)));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 1, 1)),
                          RosToGtsam(pose_graph_ptr->edges[1].pose)));

  EXPECT_EQ(0, pose_graph_ptr->edges[0].robot_from);
  EXPECT_EQ(0, pose_graph_ptr->edges[0].robot_to);
  EXPECT_EQ(0, pose_graph_ptr->edges[0].key_from);
  EXPECT_EQ(1, pose_graph_ptr->edges[0].key_to);
  EXPECT_EQ(0, pose_graph_ptr->edges[1].robot_from);
  EXPECT_EQ(0, pose_graph_ptr->edges[1].robot_to);
  EXPECT_EQ(1, pose_graph_ptr->edges[1].key_from);
  EXPECT_EQ(2, pose_graph_ptr->edges[1].key_to);
  EXPECT_EQ("world", pose_graph_ptr->edges[1].header.frame_id);

  // Check nodes
  EXPECT_EQ(size_t(3), pose_graph_ptr->nodes.size());
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(), RosToGtsam(pose_graph_ptr->nodes[0].pose)));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 1, 1)),
                          RosToGtsam(pose_graph_ptr->nodes[2].pose)));
  EXPECT_EQ(0, pose_graph_ptr->nodes[0].robot_id);
  EXPECT_EQ(0, pose_graph_ptr->nodes[2].robot_id);
  EXPECT_EQ(0, pose_graph_ptr->nodes[0].key);
  EXPECT_EQ(1, pose_graph_ptr->nodes[1].key);
  EXPECT_EQ(2, pose_graph_ptr->nodes[2].key);
}

}  // namespace kimera_pgmo
