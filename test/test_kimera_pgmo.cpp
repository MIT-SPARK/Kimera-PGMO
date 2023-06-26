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

#define WRAP_SYSTEM(command)                                      \
  {                                                               \
    int ret = system(command);                                    \
    if (ret != 0) {                                               \
      ROS_WARN_STREAM(command "failed with return code " << ret); \
    }                                                             \
  }                                                               \
  static_assert(true, "")

class KimeraPgmoTest : public ::testing::Test {
 protected:
  KimeraPgmoTest() {
    WRAP_SYSTEM("rosparam set frame_id world");
    WRAP_SYSTEM("rosparam set robot_id 0");
    WRAP_SYSTEM("rosparam set run_mode 0");
    WRAP_SYSTEM("rosparam set output_prefix test");
    WRAP_SYSTEM("rosparam set embed_trajectory_delta_t 3.0");
    WRAP_SYSTEM("rosparam set num_interp_pts 4");
    WRAP_SYSTEM("rosparam set interp_horizon 10.0");
    WRAP_SYSTEM("rosparam set rpgo/odom_trans_threshold 10.0");
    WRAP_SYSTEM("rosparam set rpgo/odom_rot_threshold 10.0");
    WRAP_SYSTEM("rosparam set rpgo/pcm_trans_threshold 10.0");
    WRAP_SYSTEM("rosparam set rpgo/pcm_rot_threshold 10.0");
    WRAP_SYSTEM("rosparam set rpgo/gnc_alpha 0");
    WRAP_SYSTEM("rosparam set add_initial_prior true");
    WRAP_SYSTEM("rosparam set enable_sparsify false");
    WRAP_SYSTEM("rosparam set covariance/odom 0.000001");
    WRAP_SYSTEM("rosparam set covariance/loop_close 0.0001");
    WRAP_SYSTEM("rosparam set covariance/prior 0.00001");
    WRAP_SYSTEM("rosparam set covariance/mesh_mesh 0.001");
    WRAP_SYSTEM("rosparam set covariance/pose_mesh 0.0001");
  }
  ~KimeraPgmoTest() {}

  void IncrementalPoseGraphCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg) {
    pgmo_.incrementalPoseGraphCallback(msg);
  }

  void FullMeshCallback(const KimeraPgmoMesh::ConstPtr& mesh_msg) {
    pgmo_.fullMeshCallback(mesh_msg);
  }

  void IncrementalMeshGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& mesh_graph_msg) {
    pgmo_.incrementalMeshGraphCallback(mesh_graph_msg);
  }

  void OptimizedPathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
    pgmo_.optimizedPathCallback(path_msg);
  }

  bool RequestMeshEdgesCallback(kimera_pgmo::RequestMeshFactors::Request& req,
                                kimera_pgmo::RequestMeshFactors::Response& res) {
    return pgmo_.requestMeshEdgesCallback(req, res);
  }

  inline void ForceOptimize() { pgmo_.forceOptimize(); }

  inline std::vector<gtsam::Pose3> getTrajectory() const { return pgmo_.trajectory_; }

  inline std::queue<size_t> getUnconnectedNodes() const {
    return pgmo_.unconnected_nodes_;
  }

  inline std::vector<ros::Time> getTimestamps() const { return pgmo_.timestamps_; }

  inline gtsam::Values getValues() const { return pgmo_.getDeformationGraphValues(); }

  inline gtsam::NonlinearFactorGraph getFactors() const {
    return pgmo_.getDeformationGraphFactors();
  }

  inline pcl::PolygonMesh getOptimizedMesh() const { return *(pgmo_.optimized_mesh_); }

  inline gtsam::NonlinearFactorGraph getConsistencyFactorsGtsam() const {
    return pgmo_.getDeformationGraphPtr()->getConsistencyFactors();
  }

  inline bool getConsistencyFactorsMsg(const size_t& robot_id,
                                       pose_graph_tools::PoseGraph* pg_mesh_msg,
                                       const size_t& vertex_index_offset) const {
    return pgmo_.getConsistencyFactors(robot_id, pg_mesh_msg, vertex_index_offset);
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
  ForceOptimize();

  std::vector<gtsam::Pose3> traj = getTrajectory();
  std::queue<size_t> unconnected_nodes = getUnconnectedNodes();
  std::vector<ros::Time> stamps = getTimestamps();
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();

  EXPECT_EQ(size_t(2), traj.size());
  EXPECT_EQ(size_t(2), unconnected_nodes.size());
  EXPECT_EQ(size_t(2), stamps.size());
  EXPECT_EQ(size_t(2), factors.size());  // 1 odom + 1 prior
  EXPECT_EQ(size_t(2), values.size());   // 2 poses

  // Briefly check values in trajectory, unconnected-nodes, stamps

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), traj[0]));
  EXPECT_EQ(0, unconnected_nodes.front());
  EXPECT_EQ(1, unconnected_nodes.back());
  EXPECT_EQ(10.2, stamps[0].toSec());

  // check values
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(),
                                  values.at<gtsam::Pose3>(gtsam::Symbol('a', 0))));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                  values.at<gtsam::Pose3>(gtsam::Symbol('a', 1))));

  // check factors
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[0]));

  gtsam::BetweenFactor<gtsam::Pose3> factor0 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[0]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                  factor0.measured()));

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(20.3), 0);
  IncrementalPoseGraphCallback(inc_graph);
  ForceOptimize();

  traj = getTrajectory();
  unconnected_nodes = getUnconnectedNodes();
  stamps = getTimestamps();
  factors = getFactors();
  values = getValues();

  EXPECT_EQ(size_t(3), traj.size());
  EXPECT_EQ(size_t(3), unconnected_nodes.size());
  EXPECT_EQ(size_t(3), stamps.size());

  EXPECT_EQ(size_t(3), values.size());   // 3 odom poses
  EXPECT_EQ(size_t(4), factors.size());  // 2 odom + 1 prior + 1 lc

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  traj[2]));
  EXPECT_EQ(2, unconnected_nodes.back());
  EXPECT_EQ(20.3, stamps[2].toSec());

  // check values
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  values.at<gtsam::Pose3>(gtsam::Symbol('a', 2))));

  // check factors (rpgo orders in: odom, special, then lc, then lndmark)
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[1]));
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[3]));

  gtsam::BetweenFactor<gtsam::Pose3> factor1 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[1]);
  gtsam::BetweenFactor<gtsam::Pose3> factor3 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[3]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)),
                                  factor1.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor3.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), factor3.back());
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor3.measured()));
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

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);

  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);
  ForceOptimize();

  // Now should have 7 values (2 nodes + 5 vertices)
  // And 28 factors (1 odom + 1 prior + 16 edges + 10 connections)
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(28), factors.size());
  EXPECT_EQ(size_t(7), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);
  ForceOptimize();

  // Now should have 13 values (3 nodes + 10 vertices)
  // And 56 factors (2 odom + 1 prior + 32 edges + 20 connections + 1 lc)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(56), factors.size());
  EXPECT_EQ(size_t(13), values.size());

  // And also add the connection of nodes and vertices
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[55]));
  gtsam::BetweenFactor<gtsam::Pose3> factor55 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[55]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor55.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor55.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), factor55.back());

  // Check deformation edge factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[3]));
  DeformationEdgeFactor factor3 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[3]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), factor3.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor3.toPoint()));
  EXPECT_EQ(gtsam::Symbol('s', 0), factor3.front());
  EXPECT_EQ(gtsam::Symbol('s', 1), factor3.back());

  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[53]));
  DeformationEdgeFactor factor53 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[53]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor53.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 0, 1), factor53.toPoint()));
  EXPECT_EQ(gtsam::Symbol('a', 2), factor53.front());
  EXPECT_EQ(gtsam::Symbol('s', 9), factor53.back());
}

TEST_F(KimeraPgmoTest, nodeToMeshConnection) {
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

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(11.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.2), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);
  ForceOptimize();

  // Now should have 8 values (3 nodes + 5 vertices)
  // And 30 factors (2 odom + 16 edges + 1 prior + 1 lc + 10 connections)
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(30), factors.size());
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
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(new pose_graph_tools::PoseGraph);
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
  KimeraPgmoMesh::Ptr full_mesh_msg(new kimera_pgmo::KimeraPgmoMesh);
  std::vector<ros::Time> full_vertex_stamps;
  for (size_t i = 0; i < full_mesh.cloud.width * full_mesh.cloud.height; i++) {
    full_vertex_stamps.push_back(ros::Time(13.0));
  }

  *full_mesh_msg = PolygonMeshToPgmoMeshMsg(0, full_mesh, full_vertex_stamps, "world");
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
  ForceOptimize();

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

  // Call full mesh callback to trigger optimization
  pcl::PolygonMesh full_mesh = createMesh(2, 2, 2);
  KimeraPgmoMesh::Ptr full_mesh_msg(new kimera_pgmo::KimeraPgmoMesh);
  std::vector<ros::Time> full_vertex_stamps;
  for (size_t i = 0; i < full_mesh.cloud.width * full_mesh.cloud.height; i++) {
    full_vertex_stamps.push_back(ros::Time(13.0));
  }

  *full_mesh_msg = PolygonMeshToPgmoMeshMsg(0, full_mesh, full_vertex_stamps, "world");
  FullMeshCallback(full_mesh_msg);

  std::vector<gtsam::Pose3> traj = getTrajectory();
  std::queue<size_t> unconnected_nodes = getUnconnectedNodes();
  std::vector<ros::Time> stamps = getTimestamps();
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();

  // check values
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 0, 0)),
                          values.at<gtsam::Pose3>(gtsam::Symbol('a', 0)),
                          1e-4));

  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(2, 0, 0)),
                          values.at<gtsam::Pose3>(gtsam::Symbol('a', 1)),
                          1e-4));

  // check factors
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[0]));
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(factors[1]));
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(factors[2]));

  gtsam::PriorFactor<gtsam::Pose3> factor1 =
      *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(factors[1]);
  gtsam::PriorFactor<gtsam::Pose3> factor2 =
      *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(factors[2]);

  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 0, 0)), factor1.prior()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor1.key());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor2.key());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(2, 0, 0)), factor2.prior()));

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
  // Call full mesh callback to trigger optimization
  FullMeshCallback(full_mesh_msg);

  traj = getTrajectory();
  unconnected_nodes = getUnconnectedNodes();
  stamps = getTimestamps();
  factors = getFactors();
  values = getValues();

  // check values
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.5, 0)),
                                  values.at<gtsam::Pose3>(gtsam::Symbol('a', 0)),
                                  1e-4));

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0.5, 0)),
                                  values.at<gtsam::Pose3>(gtsam::Symbol('a', 1)),
                                  1e-4));

  // check factors
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[0]));
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(factors[1]));
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(factors[2]));

  factor1 = *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(factors[1]);
  factor2 = *boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3> >(factors[2]);

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.5, 0)),
                                  factor1.prior()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor1.key());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor2.key());
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0.5, 0)),
                                  factor2.prior()));
}

TEST_F(KimeraPgmoTest, checkRobotIdMeshCallback) {
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  WRAP_SYSTEM("rosparam set robot_id 2");
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 2);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 2, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 2);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh2, 2, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);
  ForceOptimize();

  // Now should have 13 values (3 nodes + 10 vertices)
  // And 56 factors (2 odom + 1 prior + 32 edges + 20 connections + 1 lc)
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(56), factors.size());
  EXPECT_EQ(size_t(13), values.size());

  // And also add the connection of nodes and vertices
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[55]));
  gtsam::BetweenFactor<gtsam::Pose3> factor55 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[55]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor55.measured()));
  EXPECT_EQ(gtsam::Symbol('c', 0).key(), factor55.front());
  EXPECT_EQ(gtsam::Symbol('c', 2).key(), factor55.back());

  // Check deformation edge factors
  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[3]));
  DeformationEdgeFactor factor3 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[3]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), factor3.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor3.toPoint()));
  EXPECT_EQ(gtsam::Symbol('u', 0), factor3.front());
  EXPECT_EQ(gtsam::Symbol('u', 1), factor3.back());

  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[53]));
  DeformationEdgeFactor factor53 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[53]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor53.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 0, 1), factor53.toPoint()));
  EXPECT_EQ(gtsam::Symbol('c', 2), factor53.front());
  EXPECT_EQ(gtsam::Symbol('u', 9), factor53.back());
}

TEST_F(KimeraPgmoTest, sparseKeyFrames) {
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  WRAP_SYSTEM("rosparam set enable_sparsify true");
  WRAP_SYSTEM("rosparam set trans_node_dist 1.1");
  WRAP_SYSTEM("rosparam set rot_node_dist 2.0");
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have one node (0, 0, 0)

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);

  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);
  ForceOptimize();

  // Now should have 6 values (1 node + 5 vertices)
  // And 27 factors (1 prior + 16 edges + 10 connections)
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(27), factors.size());
  EXPECT_EQ(size_t(6), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);
  ForceOptimize();

  // Now should have 12 values (2 nodes + 10 vertices)
  // And 55 factors (1 odom + 1 prior + 32 edges + 20 connections + 1 lc)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(55), factors.size());
  EXPECT_EQ(size_t(12), values.size());

  factors.print("factors\n");

  // And also add the connection of nodes and vertices
  EXPECT_TRUE(
      boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[1]));
  gtsam::BetweenFactor<gtsam::Pose3> factor1 =
      *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(factors[1]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor1.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor1.front());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor1.back());

  EXPECT_TRUE(boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[53]));
  DeformationEdgeFactor factor53 =
      *boost::dynamic_pointer_cast<DeformationEdgeFactor>(factors[53]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor53.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 0, 1), factor53.toPoint()));
  EXPECT_EQ(gtsam::Symbol('a', 1), factor53.front());
  EXPECT_EQ(gtsam::Symbol('s', 9), factor53.back());
}

#undef WRAP_SYSTEM

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_kimera_pgmo");
  return RUN_ALL_TESTS();
}
