/**
 * @file   test_kimera_pgmo_multi.cpp
 * @brief  Unit-tests for multirobot kimera pgmo class
 * @author Yun Chang
 */
#include <ros/ros.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "gtest/gtest.h"

#include "kimera_pgmo/KimeraPgmoMulti.h"

namespace kimera_pgmo {
class KimeraPgmoMultiTest : public ::testing::Test {
 protected:
  KimeraPgmoMultiTest() : pgmo_(std::vector<size_t>{0, 1, 2}) {
    system("rosparam set frame_id world");
    system("rosparam set compression_time_horizon 10.0");
    system("rosparam set run_mode 0");
    system("rosparam set use_msg_time true");
    system("rosparam set output_prefix test");
    system("rosparam set embed_trajectory_delta_t 3.0");
    system("rosparam set d_graph_resolution 0.1");
    system("rosparam set rpgo/translation_threshold 10.0");
    system("rosparam set rpgo/rotation_threshold 10.0");
  }
  ~KimeraPgmoMultiTest() {}

  void IncrementalPoseGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& msg) {
    pgmo_.incrementalPoseGraphCallback(msg);
  }

  void FullMeshCallback(
      const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
    pgmo_.fullMeshCallback(mesh_msg);
  }

  void IncrementalMeshCallback(
      const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
    pgmo_.incrementalMeshCallback(mesh_msg);
  }

  void OptimizedPathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    pgmo_.optimizedPathCallback(path_msg);
  }

  inline std::vector<gtsam::Pose3> getTrajectory(const size_t& robot_id) const {
    return pgmo_.trajectory_.at(robot_id);
  }

  inline std::queue<size_t> getUnconnectedNodes(const size_t& robot_id) const {
    return pgmo_.unconnected_nodes_.at(robot_id);
  }

  inline std::vector<ros::Time> getTimestamps(const size_t& robot_id) const {
    return pgmo_.timestamps_.at(robot_id);
  }

  inline gtsam::Values getValues() const {
    return pgmo_.deformation_graph_.getGtsamValues();
  }

  inline gtsam::NonlinearFactorGraph getFactors() const {
    return pgmo_.deformation_graph_.getGtsamFactors();
  }

  inline pcl::PolygonMesh getOptimizedMesh(const size_t& robot_id) const {
    return pgmo_.optimized_mesh_.at(robot_id);
  }

  pose_graph_tools::PoseGraph SingleOdomGraph(const ros::Time& stamp,
                                              const size_t& robot_id) {
    pose_graph_tools::PoseGraph inc_graph;

    pose_graph_tools::PoseGraphEdge e0;
    pose_graph_tools::PoseGraphNode n0, n1;
    e0.header.stamp = stamp;
    e0.key_from = 0;
    e0.key_to = 1;
    e0.robot_from = robot_id;
    e0.robot_to = robot_id;
    e0.pose.position.x = 1;
    e0.pose.orientation.w = 1;
    e0.type = pose_graph_tools::PoseGraphEdge::ODOM;
    e0.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                     0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    n0.header.stamp = stamp;
    n0.key = 0;
    n0.robot_id = robot_id;
    n0.pose.orientation.w = 1;

    n1.header.stamp = stamp;
    n1.key = 0;
    n1.robot_id = robot_id;
    n1.pose.position.x = 1;
    n1.pose.orientation.w = 1;

    inc_graph.edges.push_back(e0);
    inc_graph.nodes.push_back(n0);
    inc_graph.nodes.push_back(n1);

    return inc_graph;
  }

  pose_graph_tools::PoseGraph OdomLoopclosureGraph(const ros::Time& stamp,
                                                   const size_t& robot_id) {
    pose_graph_tools::PoseGraph inc_graph;

    pose_graph_tools::PoseGraphEdge e1, e2;
    pose_graph_tools::PoseGraphNode n1, n2;

    e1.header.stamp = stamp;
    e1.key_from = 1;
    e1.key_to = 2;
    e1.robot_from = robot_id;
    e1.robot_to = robot_id;
    e1.pose.position.y = 1;
    e1.pose.orientation.w = 1;
    e1.type = pose_graph_tools::PoseGraphEdge::ODOM;
    e1.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                     0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    e2.header.stamp = stamp;
    e2.key_from = 0;
    e2.key_to = 2;
    e2.robot_from = robot_id;
    e2.robot_to = robot_id;
    e2.pose.position.x = 1;
    e2.pose.position.y = 1;
    e2.pose.orientation.w = 1;
    e2.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
    e2.covariance = {0.1, 0, 0,   0, 0,   0, 0, 0.1, 0, 0,   0, 0,
                     0,   0, 0.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    n1.header.stamp = stamp;
    n1.key = 0;
    n1.robot_id = robot_id;
    n1.pose.orientation.w = 1;

    n2.header.stamp = stamp;
    n2.key = 2;
    n2.robot_id = robot_id;
    n2.pose.orientation.w = 1;

    inc_graph.edges.push_back(e1);
    inc_graph.edges.push_back(e2);
    inc_graph.nodes.push_back(n1);
    inc_graph.nodes.push_back(n2);

    return inc_graph;
  }

  pose_graph_tools::PoseGraph OdomLoopclosureGraph2(const ros::Time& stamp,
                                                    const size_t& robot_id) {
    pose_graph_tools::PoseGraph inc_graph;

    pose_graph_tools::PoseGraphEdge e1, e2;
    pose_graph_tools::PoseGraphNode n1, n2;

    e1.header.stamp = stamp;
    e1.key_from = 2;
    e1.key_to = 3;
    e1.robot_from = robot_id;
    e1.robot_to = robot_id;
    e1.pose.position.z = 1;
    e1.pose.orientation.w = 1;
    e1.type = pose_graph_tools::PoseGraphEdge::ODOM;
    e1.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                     0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    e2.header.stamp = stamp;
    e2.key_from = 0;
    e2.key_to = 3;
    e2.robot_from = robot_id;
    e2.robot_to = robot_id;
    e2.pose.position.z = -1;
    e2.pose.position.x = -1;
    e2.pose.orientation.w = 1;
    e2.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
    e2.covariance = {0.1, 0, 0,   0, 0,   0, 0, 0.1, 0, 0,   0, 0,
                     0,   0, 0.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    n1.header.stamp = stamp;
    n1.key = 0;
    n1.robot_id = robot_id;
    n1.pose.orientation.w = 1;

    n2.header.stamp = stamp;
    n2.key = 3;
    n2.robot_id = robot_id;
    n2.pose.orientation.w = 1;

    inc_graph.edges.push_back(e1);
    inc_graph.edges.push_back(e2);
    inc_graph.nodes.push_back(n1);
    inc_graph.nodes.push_back(n2);

    return inc_graph;
  }

  pose_graph_tools::PoseGraph InterRobotLoopClosureGraph(
      const ros::Time& stamp,
      const size_t& robot_1,
      const size_t& robot_2) {
    pose_graph_tools::PoseGraph inc_graph;

    pose_graph_tools::PoseGraphEdge e1;

    e1.header.stamp = stamp;
    e1.key_from = 0;
    e1.key_to = 0;
    e1.robot_from = robot_1;
    e1.robot_to = robot_2;
    e1.pose.position.z = 1;
    e1.pose.orientation.w = 1;
    e1.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
    e1.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                     0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    inc_graph.edges.push_back(e1);

    return inc_graph;
  }

  pcl::PolygonMesh createMesh(double t_x, double t_y, double t_z) {
    // Create simple pcl mesh
    pcl::PolygonMesh mesh;

    pcl::PointCloud<pcl::PointXYZRGBA> ptcld;
    pcl::PointXYZRGBA v0, v1, v2, v3, v4;
    v0.x = 0 + t_x;
    v0.y = 0 + t_y;
    v0.z = 0 + t_z;
    v0.r = 23;
    v0.g = 24;
    v0.b = 122;
    v0.a = 255;

    v1.x = 1 + t_x;
    v1.y = 0 + t_y;
    v1.z = 0 + t_z;
    v1.r = 33;
    v1.g = 34;
    v1.b = 52;
    v1.a = 255;

    v2.x = 0 + t_x;
    v2.y = 1 + t_y;
    v2.z = 0 + t_z;
    v2.r = 12;
    v2.g = 144;
    v2.b = 22;
    v2.a = 255;

    v3.x = 1 + t_x;
    v3.y = 1 + t_y;
    v3.z = 0 + t_z;
    v3.r = 0;
    v3.g = 14;
    v3.b = 0;
    v3.a = 255;

    v4.x = 0 + t_x;
    v4.y = 0 + t_y;
    v4.z = 1 + t_z;
    v4.r = 144;
    v4.g = 0;
    v4.b = 12;
    v4.a = 255;

    ptcld.points.push_back(v0);
    ptcld.points.push_back(v1);
    ptcld.points.push_back(v2);
    ptcld.points.push_back(v3);
    ptcld.points.push_back(v4);
    pcl::toPCLPointCloud2(ptcld, mesh.cloud);

    pcl::Vertices tri_1, tri_2, tri_3, tri_4;
    tri_1.vertices = std::vector<uint>{0, 1, 2};
    tri_2.vertices = std::vector<uint>{1, 3, 2};
    tri_3.vertices = std::vector<uint>{0, 1, 4};
    tri_4.vertices = std::vector<uint>{0, 4, 2};
    mesh.polygons = std::vector<pcl::Vertices>{tri_1, tri_2, tri_3, tri_4};

    return mesh;
  }

  inline size_t GetNumMeshPublishers() const {
    return pgmo_.optimized_mesh_pub_.size();
  }
  inline size_t GetNumPoseGraphSubscribers() const {
    return pgmo_.pose_graph_incremental_sub_.size();
  }
  inline size_t GetNumFullMeshSubscribers() const {
    return pgmo_.full_mesh_sub_.size();
  }
  inline size_t GetNumIncMeshSubscribers() const {
    return pgmo_.incremental_mesh_sub_.size();
  }

  KimeraPgmoMulti pgmo_;
};

TEST_F(KimeraPgmoMultiTest, initialize) {
  ros::NodeHandle nh;
  bool init = pgmo_.initialize(nh);

  EXPECT_EQ(3, GetNumMeshPublishers());
  EXPECT_EQ(3, GetNumPoseGraphSubscribers());
  EXPECT_EQ(3, GetNumFullMeshSubscribers());
  EXPECT_EQ(3, GetNumIncMeshSubscribers());

  ASSERT_TRUE(init);
}

TEST_F(KimeraPgmoMultiTest, incrementalPoseGraphCallback) {
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 1);
  IncrementalPoseGraphCallback(inc_graph);

  std::vector<gtsam::Pose3> traj = getTrajectory(1);
  std::queue<size_t> unconnected_nodes = getUnconnectedNodes(1);
  std::vector<ros::Time> stamps = getTimestamps(1);
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();

  EXPECT_EQ(size_t(2), traj.size());
  EXPECT_EQ(size_t(2), unconnected_nodes.size());
  EXPECT_EQ(size_t(2), stamps.size());
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  // Briefly check values in trajectory, unconnected-nodes, stamps

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), traj[0]));
  EXPECT_EQ(0, unconnected_nodes.front());
  EXPECT_EQ(1, unconnected_nodes.back());
  EXPECT_EQ(10.2, stamps[0].toSec());

  // check values
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(), values.at<gtsam::Pose3>(gtsam::Symbol('b', 0))));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          values.at<gtsam::Pose3>(gtsam::Symbol('b', 1))));

  // check factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[0]));

  gtsam::BetweenFactor<gtsam::Pose3> factor0 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[0]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), factor0.measured()));

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(20.3), 1);
  IncrementalPoseGraphCallback(inc_graph);

  traj = getTrajectory(1);
  unconnected_nodes = getUnconnectedNodes(1);
  stamps = getTimestamps(1);
  factors = getFactors();
  values = getValues();

  EXPECT_EQ(size_t(3), traj.size());
  EXPECT_EQ(size_t(3), unconnected_nodes.size());
  EXPECT_EQ(size_t(3), stamps.size());

  EXPECT_EQ(size_t(3), values.size());
  EXPECT_EQ(size_t(3), factors.size());

  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)), traj[2]));
  EXPECT_EQ(2, unconnected_nodes.back());
  EXPECT_EQ(20.3, stamps[2].toSec());

  // check values
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                          values.at<gtsam::Pose3>(gtsam::Symbol('b', 2))));

  // check factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[1]));
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[2]));

  gtsam::BetweenFactor<gtsam::Pose3> factor1 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[1]);
  gtsam::BetweenFactor<gtsam::Pose3> factor2 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[2]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)), factor1.measured()));
  EXPECT_EQ(gtsam::Symbol('b', 0).key(), factor2.front());
  EXPECT_EQ(gtsam::Symbol('b', 2).key(), factor2.back());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)), factor2.measured()));

  //// Now try add another robot
  *inc_graph = SingleOdomGraph(ros::Time(20.4), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Check robot 1
  traj = getTrajectory(1);
  unconnected_nodes = getUnconnectedNodes(1);
  stamps = getTimestamps(1);
  EXPECT_EQ(size_t(3), traj.size());
  EXPECT_EQ(size_t(3), unconnected_nodes.size());
  EXPECT_EQ(size_t(3), stamps.size());

  // Check robot 0
  traj = getTrajectory(0);
  unconnected_nodes = getUnconnectedNodes(0);
  stamps = getTimestamps(0);
  EXPECT_EQ(size_t(2), traj.size());
  EXPECT_EQ(size_t(2), unconnected_nodes.size());
  EXPECT_EQ(size_t(2), stamps.size());

  // Check factor graph
  factors = getFactors();
  values = getValues();

  EXPECT_EQ(size_t(5), values.size());
  EXPECT_EQ(size_t(4), factors.size());

  // check values
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(), values.at<gtsam::Pose3>(gtsam::Symbol('a', 0))));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          values.at<gtsam::Pose3>(gtsam::Symbol('a', 1))));

  // check factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[2]));

  gtsam::BetweenFactor<gtsam::Pose3> factor3 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[2]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), factor3.measured()));

  //// Try add an interrobot loop closure
  *inc_graph = InterRobotLoopClosureGraph(ros::Time(21.0), 0, 1);
  IncrementalPoseGraphCallback(inc_graph);

  // Check robot 1
  traj = getTrajectory(1);
  unconnected_nodes = getUnconnectedNodes(1);
  stamps = getTimestamps(1);
  EXPECT_EQ(size_t(3), traj.size());
  EXPECT_EQ(size_t(3), unconnected_nodes.size());
  EXPECT_EQ(size_t(3), stamps.size());

  // Check robot 0
  traj = getTrajectory(0);
  unconnected_nodes = getUnconnectedNodes(0);
  stamps = getTimestamps(0);
  EXPECT_EQ(size_t(2), traj.size());
  EXPECT_EQ(size_t(2), unconnected_nodes.size());
  EXPECT_EQ(size_t(2), stamps.size());

  // Check factor graph
  factors = getFactors();
  values = getValues();

  EXPECT_EQ(size_t(5), values.size());
  EXPECT_EQ(size_t(5), factors.size());

  // check factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[3]));

  gtsam::BetweenFactor<gtsam::Pose3> factor4 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[3]);
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor4.front());
  EXPECT_EQ(gtsam::Symbol('b', 0).key(), factor4.back());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 1)), factor4.measured()));
}

TEST_F(KimeraPgmoMultiTest, incrementalMeshCallback) {
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 1);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->header.stamp = ros::Time(12.5);  // within 3 sec of pose graph msg
  mesh_msg->id = 1;
  IncrementalMeshCallback(mesh_msg);

  // Now should have 7 values (2 nodes + 5 vertices)
  // And 27 factors (16 edges + 1 odom + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(27), factors.size());
  EXPECT_EQ(size_t(7), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 1);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh2);
  mesh_msg->header.stamp = ros::Time(13.0);  // within 3 sec of pose graph msg
  mesh_msg->id = 1;
  IncrementalMeshCallback(mesh_msg);

  // Now should have 13 values (3 nodes + 10 vertices)
  // And 65 factors (32 edges + 2 odom + 20 connections + 1 lc)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(55), factors.size());
  EXPECT_EQ(size_t(13), values.size());

  // And also add the connection of nodes and vertices
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[2]));
  gtsam::BetweenFactor<gtsam::Pose3> factor2 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[2]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)), factor2.measured()));
  EXPECT_EQ(gtsam::Symbol('b', 0).key(), factor2.front());
  EXPECT_EQ(gtsam::Symbol('b', 2).key(), factor2.back());

  // Check deformation edge factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[3]));
  DeformationEdgeFactor factor3 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[3]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), factor3.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor3.toPoint()));
  EXPECT_EQ(gtsam::Symbol('t', 0), factor3.front());
  EXPECT_EQ(gtsam::Symbol('t', 1), factor3.back());

  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[53]));
  DeformationEdgeFactor factor53 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[53]);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                          factor53.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 0, 1), factor53.toPoint()));
  EXPECT_EQ(gtsam::Symbol('b', 2), factor53.front());
  EXPECT_EQ(gtsam::Symbol('t', 9), factor53.back());

  // Add mesh from another robot
  pcl::PolygonMesh mesh3 = createMesh(2, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh3);
  mesh_msg->header.stamp = ros::Time(13.0);  // within 3 sec of pose graph msg
  mesh_msg->id = 0;
  IncrementalMeshCallback(mesh_msg);

  // Now should have 18 values (3 nodes + 15 vertices)
  // And 71 factors (48 edges + 2 odom + 20 connections + 1 lc)
  factors = getFactors();
  values = getValues();

  EXPECT_EQ(size_t(71), factors.size());
  EXPECT_EQ(size_t(18), values.size());

  // Check deformation edge factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[55]));
  DeformationEdgeFactor factor55 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[55]);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 0)),
                          factor55.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(3, 0, 0), factor55.toPoint()));
  EXPECT_EQ(gtsam::Symbol('s', 0), factor55.front());
  EXPECT_EQ(gtsam::Symbol('s', 1), factor55.back());
}

TEST_F(KimeraPgmoMultiTest, nodeToMeshConnectionDeltaT) {
  // Here we check the functionality of connecting the nodes to the mesh
  // Case: mesh recieved after embed time

  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 1);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  // Add mesh from other robot so should not connect
  pcl::PolygonMesh mesh0 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh0);
  mesh_msg->header.stamp = ros::Time(13.0);  // after 3 sec of pose graph msg
  mesh_msg->id = 0;
  IncrementalMeshCallback(mesh_msg);

  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->header.stamp = ros::Time(13.5);  // after 3 sec of pose graph msg
  mesh_msg->id = 1;
  IncrementalMeshCallback(mesh_msg);

  // Now should have 12 values (2 nodes + 10 vertices)
  // And 53 factors (32 edges + 1 odom + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(43), factors.size());
  EXPECT_EQ(size_t(12), values.size());
}

TEST_F(KimeraPgmoMultiTest, nodeToMeshConnectionDelay) {
  // Here we check the functionality of connecting the nodes to the mesh
  // Case: multiple pose graph msgs recieved before mesh

  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 1);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(11.2), 1);
  IncrementalPoseGraphCallback(inc_graph);

  // At this point should have 3 nodes 3 between factors
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(3), factors.size());
  EXPECT_EQ(size_t(3), values.size());

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->id = 1;
  mesh_msg->header.stamp = ros::Time(12.2);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  // Now should have 8 values (3 nodes + 5 vertices)
  // And 49 factors (16 edges + 2 odom + 1 lc + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(29), factors.size());
  EXPECT_EQ(size_t(8), values.size());

  //// Add another robot
  // Check callback
  *inc_graph = SingleOdomGraph(ros::Time(13.2), 0);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should add two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(30), factors.size());
  EXPECT_EQ(size_t(10), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(14.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // At this point should add 1 node 2 between factors
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(32), factors.size());
  EXPECT_EQ(size_t(11), values.size());

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(0, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->id = 0;
  mesh_msg->header.stamp = ros::Time(15.2);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  // Now should have 16 values (6 nodes + 10 vertices)
  // And 98 factors (32 edges + 4 odom + 2 lc + 20 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(58), factors.size());
  EXPECT_EQ(size_t(16), values.size());
}

TEST_F(KimeraPgmoMultiTest, fullMeshCallback) {
  // Simple test with above pose graph. Mesh should not change
  // since no loop closures
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 1);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->header.stamp = ros::Time(12.5);
  mesh_msg->id = 1;
  IncrementalMeshCallback(mesh_msg);

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 1);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh2);
  mesh_msg->header.stamp = ros::Time(13.0);
  mesh_msg->id = 1;
  IncrementalMeshCallback(mesh_msg);

  // Add mesh to be deformed
  pcl::PolygonMesh full_mesh = createMesh(2, 2, 2);
  kimera_pgmo::TriangleMeshIdStamped::Ptr full_mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  full_mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(full_mesh);
  full_mesh_msg->id = 1;
  FullMeshCallback(full_mesh_msg);

  pcl::PolygonMesh optimized_mesh = getOptimizedMesh(1);

  pcl::PointCloud<pcl::PointXYZRGBA> optimized_vertices;
  pcl::fromPCLPointCloud2(optimized_mesh.cloud, optimized_vertices);

  EXPECT_EQ(size_t(5), optimized_vertices.points.size());
  EXPECT_EQ(2, optimized_vertices.points[0].x);
  EXPECT_EQ(2, optimized_vertices.points[0].y);
  EXPECT_EQ(2, optimized_vertices.points[0].z);

  EXPECT_EQ(2, optimized_vertices.points[4].x);
  EXPECT_EQ(2, optimized_vertices.points[4].y);
  EXPECT_EQ(3, optimized_vertices.points[4].z);

  // load third incremental pose graph
  *inc_graph = OdomLoopclosureGraph2(ros::Time(13.8), 1);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh3 = createMesh(2, 2, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh3);
  mesh_msg->header.stamp = ros::Time(14.0);
  mesh_msg->id = 1;
  IncrementalMeshCallback(mesh_msg);

  FullMeshCallback(full_mesh_msg);
  optimized_mesh = getOptimizedMesh(1);
  pcl::fromPCLPointCloud2(optimized_mesh.cloud, optimized_vertices);

  // Expect distortion
  EXPECT_EQ(size_t(5), optimized_vertices.points.size());
  EXPECT_NE(2, optimized_vertices.points[0].x);
  EXPECT_NE(2, optimized_vertices.points[0].y);
  EXPECT_NE(2, optimized_vertices.points[0].z);

  EXPECT_NE(2, optimized_vertices.points[4].x);
  EXPECT_NE(2, optimized_vertices.points[4].y);
  EXPECT_NE(3, optimized_vertices.points[4].z);

  // Add other robot (with no loop closures)
  *inc_graph = SingleOdomGraph(ros::Time(14.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  *inc_graph = InterRobotLoopClosureGraph(ros::Time(14.3), 0, 1);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh4 = createMesh(1, 1, 1);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh4);
  mesh_msg->header.stamp = ros::Time(14.5);
  mesh_msg->id = 0;
  IncrementalMeshCallback(mesh_msg);

  pcl::PolygonMesh mesh5 = createMesh(2, 2, 2);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh5);
  mesh_msg->header.stamp = ros::Time(14.7);
  mesh_msg->id = 0;
  IncrementalMeshCallback(mesh_msg);

  full_mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(full_mesh);
  full_mesh_msg->id = 0;
  FullMeshCallback(full_mesh_msg);

  optimized_mesh = getOptimizedMesh(0);

  pcl::PointCloud<pcl::PointXYZRGBA> full_vertices;

  pcl::fromPCLPointCloud2(optimized_mesh.cloud, optimized_vertices);
  pcl::fromPCLPointCloud2(full_mesh.cloud, full_vertices);

  // Expect no distortion
  EXPECT_EQ(size_t(5), optimized_vertices.points.size());
  EXPECT_EQ(full_vertices.points[0].x - full_vertices.points[4].x,
            optimized_vertices.points[0].x - optimized_vertices.points[4].x);
  EXPECT_EQ(full_vertices.points[0].y - full_vertices.points[4].y,
            optimized_vertices.points[0].y - optimized_vertices.points[4].y);
  EXPECT_EQ(full_vertices.points[0].z - full_vertices.points[4].z,
            optimized_vertices.points[0].z - optimized_vertices.points[4].z);
  EXPECT_EQ(full_vertices.points[1].x - full_vertices.points[3].x,
            optimized_vertices.points[1].x - optimized_vertices.points[3].x);
  EXPECT_EQ(full_vertices.points[1].y - full_vertices.points[3].y,
            optimized_vertices.points[1].y - optimized_vertices.points[3].y);
  EXPECT_EQ(full_vertices.points[1].z - full_vertices.points[3].z,
            optimized_vertices.points[1].z - optimized_vertices.points[3].z);
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_kimera_pgmo_multi");
  return RUN_ALL_TESTS();
}