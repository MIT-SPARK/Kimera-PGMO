/**
 * @file   test_kimera_pgmo.cpp
 * @brief  Unit-tests for main kimera pgmo class
 * @author Yun Chang
 */
#include <config_utilities/formatting/asl.h>
#include <config_utilities/printing.h>
#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <kimera_pgmo/utils/common_structs.h>
#include <kimera_pgmo/utils/logging.h>
#include <ros/ros.h>

#include "kimera_pgmo_ros/conversion/mesh_conversion.h"
#include "kimera_pgmo_ros/kimera_pgmo.h"
#include "test_artifacts.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::KimeraPgmoMesh;

namespace {
template <typename T, typename Ptr>
const T* cast_factor(const Ptr& ptr, bool should_throw = false) {
  if (!ptr) {
    if (should_throw) {
      throw std::runtime_error("bad pointer");
    }
    return nullptr;
  }

  auto cast = dynamic_cast<const T*>(ptr.get());
  if (!cast && should_throw) {
    throw std::runtime_error("bad cast");
  }

  return cast;
}
}  // namespace

class KimeraPgmoTest : public ::testing::Test {
 protected:
  KimeraPgmoTest() : nh("~") {
    nh.setParam("frame_id", "world");
    nh.setParam("robot_id", 0);
    nh.setParam("run_mode", "FULL");
    nh.setParam("output_prefix", "test");
    nh.setParam("embed_trajectory_delta_t", 3.0);
    nh.setParam("num_interp_pts", 4);
    nh.setParam("interp_horizon", 10.0);
    nh.setParam("optimizer/type", "KimeraRpgoOptimizer");
    nh.setParam("optimizer/solver", "LM");
    nh.setParam("optimizer/verbosity", "2");
    nh.setParam("optimizer/use_gnc", "False");
    nh.setParam("add_initial_prior", true);
    nh.setParam("covariance/odom", 0.000001);
    nh.setParam("covariance/loop_close", 0.0001);
    nh.setParam("covariance/prior", 0.00001);
    nh.setParam("covariance/mesh_mesh", 0.001);
    nh.setParam("covariance/pose_mesh", 0.0001);

    pgmo_.reset(new KimeraPgmo(nh));
  }

  ~KimeraPgmoTest() {}

  void reinit() { pgmo_.reset(new KimeraPgmo(nh)); }

  void IncrementalPoseGraphCallback(const pose_graph_tools_msgs::PoseGraph& msg) {
    pgmo_->incrementalPoseGraphCallback(msg);
  }

  void FullMeshCallback(const KimeraPgmoMesh& mesh_msg) {
    pgmo_->fullMeshCallback(mesh_msg);
  }

  void IncrementalMeshGraphCallback(const pose_graph_tools_msgs::PoseGraph& msg) {
    pgmo_->incrementalMeshGraphCallback(msg);
  }

  void OptimizedPathCallback(const nav_msgs::Path& path_msg) {
    pgmo_->optimizedPathCallback(path_msg);
  }

  std::vector<gtsam::Pose3> getTrajectory() const { return pgmo_->trajectory_; }

  std::queue<size_t> getUnconnectedNodes() const { return pgmo_->unconnected_nodes_; }

  std::vector<Timestamp> getTimestamps() const { return pgmo_->timestamps_; }

  gtsam::Values getValues() const { return pgmo_->getDeformationGraphValues(); }

  gtsam::NonlinearFactorGraph getFactors() const {
    return pgmo_->getDeformationGraphFactors();
  }

  pcl::PolygonMesh getOptimizedMesh() const { return *(pgmo_->optimized_mesh_); }

  ros::NodeHandle nh;
  std::shared_ptr<KimeraPgmo> pgmo_;
};

TEST_F(KimeraPgmoTest, incrementalPoseGraphCallback) {
  // check callback
  auto inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  std::vector<gtsam::Pose3> traj = getTrajectory();
  std::queue<size_t> unconnected_nodes = getUnconnectedNodes();
  std::vector<Timestamp> stamps = getTimestamps();
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();

  EXPECT_EQ(2u, traj.size());
  EXPECT_EQ(2u, unconnected_nodes.size());
  EXPECT_EQ(2u, stamps.size());
  EXPECT_EQ(2u, factors.size());  // 1 odom + 1 prior
  EXPECT_EQ(2u, values.size());   // 2 poses

  // Briefly check values in trajectory, unconnected-nodes, stamps

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), traj[0]));
  EXPECT_EQ(0, unconnected_nodes.front());
  EXPECT_EQ(1, unconnected_nodes.back());
  EXPECT_EQ(stampFromSec(10.2), stamps[0]);

  // check values
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(),
                                  values.at<gtsam::Pose3>(gtsam::Symbol('a', 0))));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                  values.at<gtsam::Pose3>(gtsam::Symbol('a', 1))));

  // check factors
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[1]));

  gtsam::BetweenFactor<gtsam::Pose3> factor1 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[1]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0, 0)),
                                  factor1.measured()));

  // load second incremental pose graph
  inc_graph = OdomLoopclosureGraph(ros::Time(20.3), 0);
  IncrementalPoseGraphCallback(inc_graph);

  traj = getTrajectory();
  unconnected_nodes = getUnconnectedNodes();
  stamps = getTimestamps();
  factors = getFactors();
  values = getValues();

  EXPECT_EQ(3u, traj.size());
  EXPECT_EQ(3u, unconnected_nodes.size());
  EXPECT_EQ(3u, stamps.size());

  EXPECT_EQ(3u, values.size());   // 3 odom poses
  EXPECT_EQ(4u, factors.size());  // 1 prior + 2 odom + 1 lc

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  traj[2]));
  EXPECT_EQ(2, unconnected_nodes.back());
  EXPECT_EQ(20.3, stampToSec(stamps[2]));

  // check values
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  values.at<gtsam::Pose3>(gtsam::Symbol('a', 2))));

  // check factors
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[2]));
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[3]));

  gtsam::BetweenFactor<gtsam::Pose3> factor2 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[2]);
  gtsam::BetweenFactor<gtsam::Pose3> factor3 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[3]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 1, 0)),
                                  factor2.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor3.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), factor3.back());
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor3.measured()));
}

TEST_F(KimeraPgmoTest, incrementalMeshCallback) {
  // Here we should test if the mesh is added to the deformation graph correctly
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  auto inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);

  auto mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 7 values (2 nodes + 5 vertices)
  // And 28 factors (1 prior + 1 odom + 16 edges + 10 connections)
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(28u, factors.size());
  EXPECT_EQ(7u, values.size());

  // load second incremental pose graph
  inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 13 values (3 nodes + 10 vertices)
  // And 56 factors (1 prior + 1 odom + 16 edges + 10 connections + 1 odom + 1 lc + 16
  // edges + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(56u, factors.size());
  EXPECT_EQ(13u, values.size());

  // And also add the connection of nodes and vertices
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[29]));
  gtsam::BetweenFactor<gtsam::Pose3> factor29 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[29]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor29.measured()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor29.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), factor29.back());

  // Check deformation edge factors
  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors[2]));
  DeformationEdgeFactor factor2 = *cast_factor<DeformationEdgeFactor>(factors[2]);

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor2.measurement()));
  EXPECT_EQ(gtsam::Symbol('s', 0), factor2.front());
  EXPECT_EQ(gtsam::Symbol('s', 1), factor2.back());

  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors[54]));
  DeformationEdgeFactor factor54 = *cast_factor<DeformationEdgeFactor>(factors[54]);

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, -1, 1), factor54.measurement()));
  EXPECT_EQ(gtsam::Symbol('a', 2), factor54.front());
  EXPECT_EQ(gtsam::Symbol('s', 9), factor54.back());
}

TEST_F(KimeraPgmoTest, nodeToMeshConnection) {
  // Here we check the functionality of connecting the nodes to the mesh
  // Case: multiple pose graph msgs recieved before mesh

  // Here we should test if the mesh is added to the deformation graph correctly
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  auto inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor

  // load second incremental pose graph
  inc_graph = OdomLoopclosureGraph(ros::Time(11.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  auto mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.2), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 8 values (3 nodes + 5 vertices)
  // And 30 factors (2 odom + 16 edges + 1 prior + 1 lc + 10 connections)
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(30u, factors.size());
  EXPECT_EQ(8u, values.size());
}

TEST_F(KimeraPgmoTest, fullMeshCallback) {
  // Simple test with above pose graph. Mesh should not change
  // since no loop closures
  // Here we should test if the mesh is added to the deformation graph correctly
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  auto inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  auto mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // load second incremental pose graph
  inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Add mesh to be deformed
  pcl::PolygonMesh full_mesh = createMesh(2, 2, 2);
  std::vector<Timestamp> full_vertex_stamps;
  for (size_t i = 0; i < full_mesh.cloud.width * full_mesh.cloud.height; i++) {
    full_vertex_stamps.push_back(stampFromSec(13.0));
  }

  auto full_mesh_msg = conversions::toMsg(0, full_mesh, full_vertex_stamps, "world");
  ASSERT_TRUE(full_mesh_msg);
  FullMeshCallback(*full_mesh_msg);

  pcl::PolygonMesh optimized_mesh = getOptimizedMesh();

  pcl::PointCloud<pcl::PointXYZRGBA> optimized_vertices;
  pcl::fromPCLPointCloud2(optimized_mesh.cloud, optimized_vertices);

  EXPECT_EQ(5u, optimized_vertices.points.size());
  EXPECT_EQ(2, optimized_vertices.points[0].x);
  EXPECT_EQ(2, optimized_vertices.points[0].y);
  EXPECT_EQ(2, optimized_vertices.points[0].z);

  EXPECT_EQ(2, optimized_vertices.points[4].x);
  EXPECT_EQ(2, optimized_vertices.points[4].y);
  EXPECT_EQ(3, optimized_vertices.points[4].z);

  // load third incremental pose graph
  inc_graph = OdomLoopclosureGraph2(ros::Time(13.8), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh3 = createMesh(2, 2, 0);
  mesh_graph_msg =
      processMeshToGraph(mesh3, 0, ros::Time(14.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  FullMeshCallback(*full_mesh_msg);
  optimized_mesh = getOptimizedMesh();
  pcl::fromPCLPointCloud2(optimized_mesh.cloud, optimized_vertices);

  // Expect distortion
  EXPECT_EQ(5u, optimized_vertices.points.size());
  EXPECT_NE(2, optimized_vertices.points[4].x);
  EXPECT_NE(2, optimized_vertices.points[4].y);
  EXPECT_NE(3, optimized_vertices.points[4].z);
}

TEST_F(KimeraPgmoTest, optimizedPathCallback) {
  // check callback
  auto inc_graph = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph);

  // Create a Path
  nav_msgs::Path path_msg;

  geometry_msgs::PoseStamped pose0;
  pose0.pose.position.x = 1;
  pose0.pose.orientation.w = 0;
  pose0.pose.orientation.x = 1;
  path_msg.poses.push_back(pose0);

  geometry_msgs::PoseStamped pose1;
  pose1.pose.position.x = 2;
  pose1.pose.orientation.w = 0;
  pose1.pose.orientation.x = 1;
  path_msg.poses.push_back(pose1);

  OptimizedPathCallback(path_msg);

  // Call full mesh callback to trigger optimization
  pcl::PolygonMesh full_mesh = createMesh(2, 2, 2);
  std::vector<Timestamp> full_vertex_stamps;
  for (size_t i = 0; i < full_mesh.cloud.width * full_mesh.cloud.height; i++) {
    full_vertex_stamps.push_back(stampFromSec(13.0));
  }

  auto full_mesh_msg = conversions::toMsg(0, full_mesh, full_vertex_stamps, "world");
  FullMeshCallback(*full_mesh_msg);

  std::vector<gtsam::Pose3> traj = getTrajectory();
  std::queue<size_t> unconnected_nodes = getUnconnectedNodes();
  std::vector<Timestamp> stamps = getTimestamps();
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
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[0]));
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3> >(factors[1]));
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3> >(factors[2]));

  gtsam::PriorFactor<gtsam::Pose3> factor1 =
      *cast_factor<gtsam::PriorFactor<gtsam::Pose3> >(factors[1]);
  gtsam::PriorFactor<gtsam::Pose3> factor2 =
      *cast_factor<gtsam::PriorFactor<gtsam::Pose3> >(factors[2]);

  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 0, 0)), factor1.prior()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor1.key());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor2.key());
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(2, 0, 0)), factor2.prior()));

  // Create a Path2
  path_msg.poses.clear();

  pose0.pose.position.x = 0;
  pose0.pose.position.y = 0.5;
  pose0.pose.orientation.w = 1;
  pose0.pose.orientation.x = 0;
  path_msg.poses.push_back(pose0);

  pose1.pose.position.x = 1;
  pose1.pose.position.y = 0.5;
  pose1.pose.orientation.w = 1;
  pose1.pose.orientation.x = 0;
  path_msg.poses.push_back(pose1);

  OptimizedPathCallback(path_msg);
  // Call full mesh callback to trigger optimization
  FullMeshCallback(*full_mesh_msg);

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
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[0]));
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3> >(factors[1]));
  EXPECT_TRUE(cast_factor<gtsam::PriorFactor<gtsam::Pose3> >(factors[2]));

  factor1 = *cast_factor<gtsam::PriorFactor<gtsam::Pose3> >(factors[1]);
  factor2 = *cast_factor<gtsam::PriorFactor<gtsam::Pose3> >(factors[2]);

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0.5, 0)),
                                  factor1.prior()));
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor1.key());
  EXPECT_EQ(gtsam::Symbol('a', 1).key(), factor2.key());
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 0.5, 0)),
                                  factor2.prior()));
}

TEST_F(KimeraPgmoTest, checkRobotIdMeshCallback) {
  nh.setParam("robot_id", 2);

  reinit();

  // Here we should test if the mesh is added to the deformation graph correctly
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  auto inc_graph = SingleOdomGraph(ros::Time(10.2), 2);
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  auto mesh_graph_msg =
      processMeshToGraph(mesh1, 2, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // load second incremental pose graph
  inc_graph = OdomLoopclosureGraph(ros::Time(12.8), 2);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_graph_msg =
      processMeshToGraph(mesh2, 2, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // Now should have 13 values (3 nodes + 10 vertices)
  // And 56 factors (1 prior + 1 odom + 16 edges + 10 connections + 1 odom + 1 lc + 16
  // edges + 10 connections)
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(56u, factors.size());
  EXPECT_EQ(13u, values.size());

  // And also add the connection of nodes and vertices
  EXPECT_TRUE(cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[29]));
  gtsam::BetweenFactor<gtsam::Pose3> factor29 =
      *cast_factor<gtsam::BetweenFactor<gtsam::Pose3> >(factors[29]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                                  factor29.measured()));
  EXPECT_EQ(gtsam::Symbol('c', 0).key(), factor29.front());
  EXPECT_EQ(gtsam::Symbol('c', 2).key(), factor29.back());

  // Check deformation edge factors
  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors[2]));
  DeformationEdgeFactor factor2 = *cast_factor<DeformationEdgeFactor>(factors[2]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, 0, 0), factor2.measurement()));
  EXPECT_EQ(gtsam::Symbol('u', 0), factor2.front());
  EXPECT_EQ(gtsam::Symbol('u', 1), factor2.back());

  EXPECT_TRUE(cast_factor<DeformationEdgeFactor>(factors[54]));
  DeformationEdgeFactor factor54 = *cast_factor<DeformationEdgeFactor>(factors[54]);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(1, -1, 1), factor54.measurement()));
  EXPECT_EQ(gtsam::Symbol('c', 2), factor54.front());
  EXPECT_EQ(gtsam::Symbol('u', 9), factor54.back());
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  logging::Logger::addSink("cout", std::make_shared<logging::CoutSink>());
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_ros-test_kimera_pgmo");
  return RUN_ALL_TESTS();
}
