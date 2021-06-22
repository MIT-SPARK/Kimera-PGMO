/**
 * @file   test_kimera_pgmo.cpp
 * @brief  Unit-tests for main kimera pgmo class
 * @author Yun Chang
 */
#include <ros/ros.h>

#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include "gtest/gtest.h"

#include "kimera_pgmo/KimeraPgmo.h"
#include "kimera_pgmo/utils/CommonStructs.h"
#include "test_artifacts.h"

namespace kimera_pgmo {

class KimeraPgmoTest : public ::testing::Test {
 protected:
  KimeraPgmoTest() {
    system("rosparam set frame_id world");
    system("rosparam set robot_id 0");
    system("rosparam set run_mode 0");
    system("rosparam set use_msg_time true");
    system("rosparam set output_prefix test");
    system("rosparam set embed_trajectory_delta_t 3.0");
    system("rosparam set rpgo/translation_threshold 10.0");
    system("rosparam set rpgo/rotation_threshold 10.0");
  }
  ~KimeraPgmoTest() {}

  void IncrementalPoseGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& msg) {
    pgmo_.incrementalPoseGraphCallback(msg);
  }

  void FullMeshCallback(
      const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
    pgmo_.fullMeshCallback(mesh_msg);
    ros::TimerEvent timer_event;
    timer_event.current_real = ros::Time::now();
    timer_event.current_expected = ros::Time::now();
    pgmo_.processTimerCallback(timer_event);
  }

  void IncrementalMeshGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& mesh_graph_msg) {
    pgmo_.incrementalMeshGraphCallback(mesh_graph_msg);
  }

  void OptimizedPathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    pgmo_.optimizedPathCallback(path_msg);
  }

  bool RequestMeshEdgesCallback(
      kimera_pgmo::RequestMeshFactors::Request& req,
      kimera_pgmo::RequestMeshFactors::Response& res) {
    return pgmo_.requestMeshEdgesCallback(req, res);
  }

  inline std::vector<gtsam::Pose3> getTrajectory() const {
    return pgmo_.trajectory_;
  }

  inline std::queue<size_t> getUnconnectedNodes() const {
    return pgmo_.unconnected_nodes_;
  }

  inline std::vector<ros::Time> getTimestamps() const {
    return pgmo_.timestamps_;
  }

  inline gtsam::Values getValues() const {
    return pgmo_.getDeformationGraphValues();
  }

  inline gtsam::NonlinearFactorGraph getFactors() const {
    return pgmo_.getDeformationGraphFactors();
  }

  inline pcl::PolygonMesh getOptimizedMesh() const {
    return *(pgmo_.optimized_mesh_);
  }

  inline gtsam::NonlinearFactorGraph getConsistencyFactorsGtsam() const {
    return pgmo_.getDeformationGraphPtr()->getConsistencyFactors();
  }

  inline bool getConsistencyFactorsMsg(
      const size_t& robot_id,
      pose_graph_tools::PoseGraph* pg_mesh_msg,
      const size_t& vertex_index_offset) const {
    return pgmo_.getConsistencyFactors(
        robot_id, pg_mesh_msg, vertex_index_offset);
  }

  KimeraPgmo pgmo_;
};

TEST_F(KimeraPgmoTest, initialize) {
  ros::NodeHandle nh;
  bool init = pgmo_.initialize(nh);

  ASSERT_TRUE(init);
}

TEST_F(KimeraPgmoTest, incrementalPoseGraphCallback) {
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  std::vector<gtsam::Pose3> traj = getTrajectory();
  std::queue<size_t> unconnected_nodes = getUnconnectedNodes();
  std::vector<ros::Time> stamps = getTimestamps();
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
      gtsam::Pose3(), values.at<gtsam::Pose3>(gtsam::Symbol('a', 0))));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                          values.at<gtsam::Pose3>(gtsam::Symbol('a', 1))));

  // check factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[0]));

  gtsam::BetweenFactor<gtsam::Pose3> factor0 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[0]);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)), factor0.measured()));

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(20.3), 0);
  IncrementalPoseGraphCallback(inc_graph);

  traj = getTrajectory();
  unconnected_nodes = getUnconnectedNodes();
  stamps = getTimestamps();
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
                          values.at<gtsam::Pose3>(gtsam::Symbol('a', 2))));

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
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor2.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), factor2.back());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)), factor2.measured()));
}

TEST_F(KimeraPgmoTest, incrementalMeshCallback) {
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);

  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(
      new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 7 values (2 nodes + 5 vertices)
  // And 27 factors (16 edges + 1 odom + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(27), factors.size());
  EXPECT_EQ(size_t(7), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 13 values (3 nodes + 10 vertices)
  // And 55 factors (32 edges + 2 odom + 20 connections + 1 lc)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(55), factors.size());
  EXPECT_EQ(size_t(13), values.size());

  // And also add the connection of nodes and vertices
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[54]));
  gtsam::BetweenFactor<gtsam::Pose3> factor54 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[54]);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                          factor54.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor54.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), factor54.back());

  // Check deformation edge factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[2]));
  DeformationEdgeFactor factor2 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[2]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), factor2.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor2.toPoint()));
  EXPECT_EQ(gtsam::Symbol('s', 0), factor2.front());
  EXPECT_EQ(gtsam::Symbol('s', 1), factor2.back());

  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[52]));
  DeformationEdgeFactor factor52 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[52]);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                          factor52.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 0, 1), factor52.toPoint()));
  EXPECT_EQ(gtsam::Symbol('a', 2), factor52.front());
  EXPECT_EQ(gtsam::Symbol('s', 9), factor52.back());
}

TEST_F(KimeraPgmoTest, nodeToMeshConnectionDeltaT) {
  // Here we check the functionality of connecting the nodes to the mesh
  // Case: mesh recieved after embed time

  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(
      new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(13.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 7 values (2 nodes + 5 vertices)
  // And 27 factors (16 edges + 1 odom + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(27), factors.size());
  EXPECT_EQ(size_t(7), values.size());
}

TEST_F(KimeraPgmoTest, nodeToMeshConnectionDelay) {
  // Here we check the functionality of connecting the nodes to the mesh
  // Case: multiple pose graph msgs recieved before mesh

  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(11.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // At this point should have 3 nodes 3 between factors
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(3), factors.size());
  EXPECT_EQ(size_t(3), values.size());

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(
      new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.2), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 8 values (3 nodes + 5 vertices)
  // And 29 factors (16 edges + 2 odom + 1 lc + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(29), factors.size());
  EXPECT_EQ(size_t(8), values.size());
}

TEST_F(KimeraPgmoTest, fullMeshCallback) {
  // Simple test with above pose graph. Mesh should not change
  // since no loop closures
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(
      new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Add mesh to be deformed
  pcl::PolygonMesh full_mesh = createMesh(2, 2, 2);
  kimera_pgmo::TriangleMeshIdStamped::Ptr full_mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  full_mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(full_mesh);
  FullMeshCallback(full_mesh_msg);

  pcl::PolygonMesh optimized_mesh = getOptimizedMesh();

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
  *inc_graph = OdomLoopclosureGraph2(ros::Time(13.8), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh3 = createMesh(2, 2, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh3, 0, ros::Time(14.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  FullMeshCallback(full_mesh_msg);
  optimized_mesh = getOptimizedMesh();
  pcl::fromPCLPointCloud2(optimized_mesh.cloud, optimized_vertices);

  // Expect distortion
  EXPECT_EQ(size_t(5), optimized_vertices.points.size());
  EXPECT_NE(2, optimized_vertices.points[4].x);
  EXPECT_NE(2, optimized_vertices.points[4].y);
  EXPECT_NE(3, optimized_vertices.points[4].z);
}

TEST_F(KimeraPgmoTest, optimizedPathCallback) {
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Create a Path
  nav_msgs::Path::Ptr path_msg(new nav_msgs::Path);

  geometry_msgs::PoseStamped pose0;
  pose0.pose.position.x = 1;
  pose0.pose.orientation.w = 0;
  pose0.pose.orientation.x = 1;
  path_msg->poses.push_back(pose0);

  geometry_msgs::PoseStamped pose1;
  pose1.pose.position.x = 2;
  pose1.pose.orientation.w = 0;
  pose1.pose.orientation.x = 1;
  path_msg->poses.push_back(pose1);

  OptimizedPathCallback(path_msg);

  std::vector<gtsam::Pose3> traj = getTrajectory();
  std::queue<size_t> unconnected_nodes = getUnconnectedNodes();
  std::vector<ros::Time> stamps = getTimestamps();
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();

  // check values
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 0, 0)),
      values.at<gtsam::Pose3>(gtsam::Symbol('a', 0)),
      1e-4));

  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(2, 0, 0)),
      values.at<gtsam::Pose3>(gtsam::Symbol('a', 1)),
      1e-4));

  // check factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[0]));
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[1]));
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[2]));

  gtsam::PriorFactor<gtsam::Pose3> factor1 =
      *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
          factors[1]);
  gtsam::PriorFactor<gtsam::Pose3> factor2 =
      *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
          factors[2]);

  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 0, 0)),
      factor1.prior()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor1.key());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor2.key());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(2, 0, 0)),
      factor2.prior()));

  // Create a Path2
  path_msg->poses.clear();

  pose0.pose.position.x = 0;
  pose0.pose.position.y = 0.5;
  pose0.pose.orientation.w = 1;
  pose0.pose.orientation.x = 0;
  path_msg->poses.push_back(pose0);

  pose1.pose.position.x = 1;
  pose1.pose.position.y = 0.5;
  pose1.pose.orientation.w = 1;
  pose1.pose.orientation.x = 0;
  path_msg->poses.push_back(pose1);

  OptimizedPathCallback(path_msg);

  traj = getTrajectory();
  unconnected_nodes = getUnconnectedNodes();
  stamps = getTimestamps();
  factors = getFactors();
  values = getValues();

  // check values
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.5, 0)),
                          values.at<gtsam::Pose3>(gtsam::Symbol('a', 0)),
                          1e-4));

  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0.5, 0)),
                          values.at<gtsam::Pose3>(gtsam::Symbol('a', 1)),
                          1e-4));

  // check factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[0]));
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[1]));
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[2]));

  factor1 = *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[1]);
  factor2 = *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(
      factors[2]);

  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.5, 0)), factor1.prior()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor1.key());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor2.key());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0.5, 0)), factor2.prior()));
}

TEST_F(KimeraPgmoTest, checkRobotIdMeshCallback) {
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  system("rosparam set robot_id 2");
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 2);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(
      new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 2, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 7 values (2 nodes + 5 vertices)
  // And 27 factors (16 edges + 1 odom + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(27), factors.size());
  EXPECT_EQ(size_t(7), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 2);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh2, 2, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 13 values (3 nodes + 10 vertices)
  // And 55 factors (32 edges + 2 odom + 20 connections + 1 lc)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(55), factors.size());
  EXPECT_EQ(size_t(13), values.size());

  // And also add the connection of nodes and vertices
  EXPECT_TRUE(boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
      factors[54]));
  gtsam::BetweenFactor<gtsam::Pose3> factor54 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
          factors[54]);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                          factor54.measured()));
  EXPECT_EQ(gtsam::Symbol('c', 0).key(), factor54.front());
  EXPECT_EQ(gtsam::Symbol('c', 2).key(), factor54.back());

  // Check deformation edge factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[2]));
  DeformationEdgeFactor factor2 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[2]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), factor2.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor2.toPoint()));
  EXPECT_EQ(gtsam::Symbol('u', 0), factor2.front());
  EXPECT_EQ(gtsam::Symbol('u', 1), factor2.back());

  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[52]));
  DeformationEdgeFactor factor52 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[52]);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                          factor52.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 0, 1), factor52.toPoint()));
  EXPECT_EQ(gtsam::Symbol('c', 2), factor52.front());
  EXPECT_EQ(gtsam::Symbol('u', 9), factor52.back());
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_kimera_pgmo");
  return RUN_ALL_TESTS();
}