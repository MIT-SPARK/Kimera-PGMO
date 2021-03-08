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

namespace kimera_pgmo {

class KimeraPgmoTest : public ::testing::Test {
 protected:
  KimeraPgmoTest() {
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
  ~KimeraPgmoTest() {}

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
    return pgmo_.deformation_graph_.getGtsamValues();
  }

  inline gtsam::NonlinearFactorGraph getFactors() const {
    return pgmo_.deformation_graph_.getGtsamFactors();
  }

  inline pcl::PolygonMesh getOptimizedMesh() const {
    return pgmo_.optimized_mesh_;
  }

  inline gtsam::NonlinearFactorGraph getConsistencyFactorsGtsam() const {
    return pgmo_.deformation_graph_.getConsistencyFactors();
  }

  inline bool getConsistencyFactorsMsg(
      const size_t& robot_id,
      std::vector<pose_graph_tools::PoseGraphEdge>* edges,
      const size_t& vertex_index_offset) const {
    return pgmo_.getConsistencyFactors(robot_id, edges, vertex_index_offset);
  }

  pose_graph_tools::PoseGraph SingleOdomGraph(const ros::Time& stamp) {
    pose_graph_tools::PoseGraph inc_graph;

    pose_graph_tools::PoseGraphEdge e0;
    pose_graph_tools::PoseGraphNode n0, n1;
    e0.header.stamp = stamp;
    e0.key_from = 0;
    e0.key_to = 1;
    e0.pose.position.x = 1;
    e0.pose.orientation.w = 1;
    e0.type = pose_graph_tools::PoseGraphEdge::ODOM;
    e0.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                     0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    n0.header.stamp = stamp;
    n0.key = 0;
    n0.pose.orientation.w = 1;

    n1.header.stamp = stamp;
    n1.key = 0;
    n1.pose.position.x = 1;
    n1.pose.orientation.w = 1;

    inc_graph.edges.push_back(e0);
    inc_graph.nodes.push_back(n0);
    inc_graph.nodes.push_back(n1);

    return inc_graph;
  }

  pose_graph_tools::PoseGraph OdomLoopclosureGraph(const ros::Time& stamp) {
    pose_graph_tools::PoseGraph inc_graph;

    pose_graph_tools::PoseGraphEdge e1, e2;
    pose_graph_tools::PoseGraphNode n1, n2;

    e1.header.stamp = stamp;
    e1.key_from = 1;
    e1.key_to = 2;
    e1.pose.position.y = 1;
    e1.pose.orientation.w = 1;
    e1.type = pose_graph_tools::PoseGraphEdge::ODOM;
    e1.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                     0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    e2.header.stamp = stamp;
    e2.key_from = 0;
    e2.key_to = 2;
    e2.pose.position.x = 1;
    e2.pose.position.y = 1;
    e2.pose.orientation.w = 1;
    e2.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
    e2.covariance = {0.1, 0, 0,   0, 0,   0, 0, 0.1, 0, 0,   0, 0,
                     0,   0, 0.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    n1.header.stamp = stamp;
    n1.key = 0;
    n1.pose.orientation.w = 1;

    n2.header.stamp = stamp;
    n2.key = 2;
    n2.pose.orientation.w = 1;

    inc_graph.edges.push_back(e1);
    inc_graph.edges.push_back(e2);
    inc_graph.nodes.push_back(n1);
    inc_graph.nodes.push_back(n2);

    return inc_graph;
  }

  pose_graph_tools::PoseGraph OdomLoopclosureGraph2(const ros::Time& stamp) {
    pose_graph_tools::PoseGraph inc_graph;

    pose_graph_tools::PoseGraphEdge e1, e2;
    pose_graph_tools::PoseGraphNode n1, n2;

    e1.header.stamp = stamp;
    e1.key_from = 2;
    e1.key_to = 3;
    e1.pose.position.z = 1;
    e1.pose.orientation.w = 1;
    e1.type = pose_graph_tools::PoseGraphEdge::ODOM;
    e1.covariance = {3.1, 0, 0,   0, 0,   0, 0, 3.1, 0, 0,   0, 0,
                     0,   0, 3.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    e2.header.stamp = stamp;
    e2.key_from = 0;
    e2.key_to = 3;
    e2.pose.position.z = -1;
    e2.pose.position.x = -1;
    e2.pose.orientation.w = 1;
    e2.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
    e2.covariance = {0.1, 0, 0,   0, 0,   0, 0, 0.1, 0, 0,   0, 0,
                     0,   0, 0.1, 0, 0,   0, 0, 0,   0, 0.1, 0, 0,
                     0,   0, 0,   0, 0.1, 0, 0, 0,   0, 0,   0, 0.1};

    n1.header.stamp = stamp;
    n1.key = 0;
    n1.pose.orientation.w = 1;

    n2.header.stamp = stamp;
    n2.key = 3;
    n2.pose.orientation.w = 1;

    inc_graph.edges.push_back(e1);
    inc_graph.edges.push_back(e2);
    inc_graph.nodes.push_back(n1);
    inc_graph.nodes.push_back(n2);

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
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
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
  *inc_graph = OdomLoopclosureGraph(ros::Time(20.3));
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

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
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
  IncrementalMeshCallback(mesh_msg);

  // Now should have 7 values (2 nodes + 5 vertices)
  // And 27 factors (16 edges + 1 odom + 10 connections)
  factors = getFactors();
  values = getValues();
  EXPECT_EQ(size_t(27), factors.size());
  EXPECT_EQ(size_t(7), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh2);
  mesh_msg->header.stamp = ros::Time(13.0);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  // Now should have 13 values (3 nodes + 10 vertices)
  // And 55 factors (32 edges + 2 odom + 20 connections + 1 lc)
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
  EXPECT_EQ(gtsam::Symbol('a', 0).key(), factor2.front());
  EXPECT_EQ(gtsam::Symbol('a', 2).key(), factor2.back());

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
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(1, 1, 0)),
                          factor53.fromPose()));
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(2, 0, 1), factor53.toPoint()));
  EXPECT_EQ(gtsam::Symbol('a', 2), factor53.front());
  EXPECT_EQ(gtsam::Symbol('s', 9), factor53.back());
}

TEST_F(KimeraPgmoTest, nodeToMeshConnectionDeltaT) {
  // Here we check the functionality of connecting the nodes to the mesh
  // Case: mesh recieved after embed time

  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->header.stamp = ros::Time(13.5);  // after 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

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

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
  IncrementalPoseGraphCallback(inc_graph);
  // At this point should have two nodes (0, 0, 0), (1, 0, 0) and a between
  // factor
  gtsam::NonlinearFactorGraph factors = getFactors();
  gtsam::Values values = getValues();
  EXPECT_EQ(size_t(1), factors.size());
  EXPECT_EQ(size_t(2), values.size());

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(11.2));
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
  mesh_msg->header.stamp = ros::Time(12.2);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

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

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->header.stamp = ros::Time(12.5);
  IncrementalMeshCallback(mesh_msg);

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh2);
  mesh_msg->header.stamp = ros::Time(13.0);
  IncrementalMeshCallback(mesh_msg);

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
  *inc_graph = OdomLoopclosureGraph2(ros::Time(13.8));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh3 = createMesh(2, 2, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh3);
  mesh_msg->header.stamp = ros::Time(14.0);
  IncrementalMeshCallback(mesh_msg);

  FullMeshCallback(full_mesh_msg);
  optimized_mesh = getOptimizedMesh();
  pcl::fromPCLPointCloud2(optimized_mesh.cloud, optimized_vertices);

  // Expect distortion
  EXPECT_EQ(size_t(5), optimized_vertices.points.size());
  EXPECT_NE(2, optimized_vertices.points[0].x);
  EXPECT_NE(2, optimized_vertices.points[0].y);
  EXPECT_NE(2, optimized_vertices.points[0].z);

  EXPECT_NE(2, optimized_vertices.points[4].x);
  EXPECT_NE(2, optimized_vertices.points[4].y);
  EXPECT_NE(3, optimized_vertices.points[4].z);
}

TEST_F(KimeraPgmoTest, optimizedPathCallback) {
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
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

TEST_F(KimeraPgmoTest, getConsistencyFactors) {
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->header.stamp = ros::Time(12.5);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh2);
  mesh_msg->header.stamp = ros::Time(13.0);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  gtsam::NonlinearFactorGraph consistency_factors =
      getConsistencyFactorsGtsam();
  std::vector<pose_graph_tools::PoseGraphEdge> consistency_edges;

  // First test with no offset
  getConsistencyFactorsMsg(0, &consistency_edges, 0);

  // Arbitrary noise
  gtsam::Vector6 precisions_6;
  precisions_6.head<3>().setConstant(0.0);
  precisions_6.tail<3>().setConstant(1e3);
  static const gtsam::SharedNoiseModel& between_noise =
      gtsam::noiseModel::Diagonal::Precisions(precisions_6);

  // Iterate and check
  for (size_t i = 0; i < consistency_factors.size(); i++) {
    // Compare the two
    DeformationEdgeFactor dedge_factor =
        *boost::dynamic_pointer_cast<DeformationEdgeFactor>(
            consistency_factors[i]);
    // Create the between factor that is equivalent to the pose graph edge
    pose_graph_tools::PoseGraphEdge e = consistency_edges[i];
    gtsam::Key front, back;
    switch (e.type) {
      case pose_graph_tools::PoseGraphEdge::MESH: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from);
        back =
            gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to), e.key_to);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::POSE_MESH: {
        front = gtsam::Symbol(robot_id_to_prefix.at(e.robot_from), e.key_from);
        back =
            gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to), e.key_to);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::MESH_POSE: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from);
        back = gtsam::Symbol(robot_id_to_prefix.at(e.robot_to), e.key_to);
        break;
      }
    }
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(
        front, back, RosToGtsam(e.pose), between_noise);

    // Create perturbed poses
    gtsam::Pose3 pose_front = dedge_factor.fromPose() *
                              gtsam::Pose3(gtsam::Rot3::Rodrigues(0.01, 0, 0),
                                           gtsam::Point3(0, 0, 0.001));
    gtsam::Pose3 pose_back =
        gtsam::Pose3(gtsam::Rot3(), dedge_factor.toPoint()) *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0, 0, 0.01),
                     gtsam::Point3(-0.001, 0, 0));

    // Check errors are equal
    gtsam::Vector betweenError =
        between_factor.evaluateError(pose_front, pose_back);

    gtsam::Vector dedgeError =
        dedge_factor.evaluateError(pose_front, pose_back);

    EXPECT_NEAR(-dedgeError(0), betweenError(3), 1e-4);
    EXPECT_NEAR(-dedgeError(1), betweenError(4), 1e-4);
    EXPECT_NEAR(-dedgeError(2), betweenError(5), 1e-4);

    // Check keys are equal
    EXPECT_EQ(dedge_factor.front(), between_factor.front());
    EXPECT_EQ(dedge_factor.back(), between_factor.back());

    // Check Covariance
    EXPECT_EQ(0, e.covariance[0]);
    EXPECT_EQ(0, e.covariance[7]);
    EXPECT_EQ(0, e.covariance[14]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[21]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[28]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[35]);
  }
}

TEST_F(KimeraPgmoTest, RequestMeshEdgesCallback) {
  // Test no reindex
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->header.stamp = ros::Time(12.5);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh2);
  mesh_msg->header.stamp = ros::Time(13.0);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  kimera_pgmo::RequestMeshFactors::Request request;
  kimera_pgmo::RequestMeshFactors::Response response;

  request.robot_id = 1;  // first test with wrong robot id

  EXPECT_FALSE(RequestMeshEdgesCallback(request, response));

  request.robot_id = 0;              // test with correct robot id
  request.reindex_vertices = false;  // do not reindex

  EXPECT_TRUE(RequestMeshEdgesCallback(request, response));
  gtsam::NonlinearFactorGraph consistency_factors =
      getConsistencyFactorsGtsam();
  std::vector<pose_graph_tools::PoseGraphEdge> consistency_edges =
      response.mesh_factors;

  // Arbitrary noise
  gtsam::Vector6 precisions_6;
  precisions_6.head<3>().setConstant(0.0);
  precisions_6.tail<3>().setConstant(1e3);
  static const gtsam::SharedNoiseModel& between_noise =
      gtsam::noiseModel::Diagonal::Precisions(precisions_6);

  // Iterate and check
  for (size_t i = 0; i < consistency_factors.size(); i++) {
    // Compare the two
    DeformationEdgeFactor dedge_factor =
        *boost::dynamic_pointer_cast<DeformationEdgeFactor>(
            consistency_factors[i]);
    // Create the between factor that is equivalent to the pose graph edge
    pose_graph_tools::PoseGraphEdge e = consistency_edges[i];
    gtsam::Key front, back;
    switch (e.type) {
      case pose_graph_tools::PoseGraphEdge::MESH: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from);
        back =
            gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to), e.key_to);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::POSE_MESH: {
        front = gtsam::Symbol(robot_id_to_prefix.at(e.robot_from), e.key_from);
        back =
            gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to), e.key_to);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::MESH_POSE: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from);
        back = gtsam::Symbol(robot_id_to_prefix.at(e.robot_to), e.key_to);
        break;
      }
    }
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(
        front, back, RosToGtsam(e.pose), between_noise);

    // Create perturbed poses
    gtsam::Pose3 pose_front = dedge_factor.fromPose() *
                              gtsam::Pose3(gtsam::Rot3::Rodrigues(0.01, 0, 0),
                                           gtsam::Point3(0, 0, 0.001));
    gtsam::Pose3 pose_back =
        gtsam::Pose3(gtsam::Rot3(), dedge_factor.toPoint()) *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0, 0, 0.01),
                     gtsam::Point3(-0.001, 0, 0));

    // Check errors are equal
    gtsam::Vector betweenError =
        between_factor.evaluateError(pose_front, pose_back);

    gtsam::Vector dedgeError =
        dedge_factor.evaluateError(pose_front, pose_back);

    EXPECT_NEAR(-dedgeError(0), betweenError(3), 1e-4);
    EXPECT_NEAR(-dedgeError(1), betweenError(4), 1e-4);
    EXPECT_NEAR(-dedgeError(2), betweenError(5), 1e-4);

    // Check keys are equal
    EXPECT_EQ(dedge_factor.front(), between_factor.front());
    EXPECT_EQ(dedge_factor.back(), between_factor.back());

    // Check Covariance
    EXPECT_EQ(0, e.covariance[0]);
    EXPECT_EQ(0, e.covariance[7]);
    EXPECT_EQ(0, e.covariance[14]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[21]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[28]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[35]);
  }
}

TEST_F(KimeraPgmoTest, RequestMeshEdgesCallbackReindex) {
  // Test reindex
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  kimera_pgmo::TriangleMeshIdStamped::Ptr mesh_msg(
      new kimera_pgmo::TriangleMeshIdStamped);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh1);
  mesh_msg->header.stamp = ros::Time(12.5);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  // load second incremental pose graph
  *inc_graph = OdomLoopclosureGraph(ros::Time(12.8));
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_msg->mesh = PolygonMeshToTriangleMeshMsg(mesh2);
  mesh_msg->header.stamp = ros::Time(13.0);  // within 3 sec of pose graph msg
  IncrementalMeshCallback(mesh_msg);

  kimera_pgmo::RequestMeshFactors::Request request;
  kimera_pgmo::RequestMeshFactors::Response response;

  request.robot_id = 1;             // first test with wrong robot id
  request.reindex_vertices = true;  // do not reindex

  EXPECT_FALSE(RequestMeshEdgesCallback(request, response));

  request.robot_id = 0;  // test with correct robot id

  EXPECT_TRUE(RequestMeshEdgesCallback(request, response));
  gtsam::NonlinearFactorGraph consistency_factors =
      getConsistencyFactorsGtsam();
  std::vector<pose_graph_tools::PoseGraphEdge> consistency_edges =
      response.mesh_factors;

  // Arbitrary noise
  gtsam::Vector6 precisions_6;
  precisions_6.head<3>().setConstant(0.0);
  precisions_6.tail<3>().setConstant(1e3);
  static const gtsam::SharedNoiseModel& between_noise =
      gtsam::noiseModel::Diagonal::Precisions(precisions_6);
  size_t num_poses = getTrajectory().size();
  // Iterate and check
  for (size_t i = 0; i < consistency_factors.size(); i++) {
    // Compare the two
    DeformationEdgeFactor dedge_factor =
        *boost::dynamic_pointer_cast<DeformationEdgeFactor>(
            consistency_factors[i]);
    // Create the between factor that is equivalent to the pose graph edge
    pose_graph_tools::PoseGraphEdge e = consistency_edges[i];
    gtsam::Key front, back;
    switch (e.type) {
      case pose_graph_tools::PoseGraphEdge::MESH: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from - num_poses);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to),
                             e.key_to - num_poses);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::POSE_MESH: {
        front = gtsam::Symbol(robot_id_to_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to),
                             e.key_to - num_poses);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::MESH_POSE: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from - num_poses);
        back = gtsam::Symbol(robot_id_to_prefix.at(e.robot_to), e.key_to);
        break;
      }
    }
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(
        front, back, RosToGtsam(e.pose), between_noise);

    // Create perturbed poses
    gtsam::Pose3 pose_front = dedge_factor.fromPose() *
                              gtsam::Pose3(gtsam::Rot3::Rodrigues(0.01, 0, 0),
                                           gtsam::Point3(0, 0, 0.001));
    gtsam::Pose3 pose_back =
        gtsam::Pose3(gtsam::Rot3(), dedge_factor.toPoint()) *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0, 0, 0.01),
                     gtsam::Point3(-0.001, 0, 0));

    // Check errors are equal
    gtsam::Vector betweenError =
        between_factor.evaluateError(pose_front, pose_back);

    gtsam::Vector dedgeError =
        dedge_factor.evaluateError(pose_front, pose_back);

    EXPECT_NEAR(-dedgeError(0), betweenError(3), 1e-4);
    EXPECT_NEAR(-dedgeError(1), betweenError(4), 1e-4);
    EXPECT_NEAR(-dedgeError(2), betweenError(5), 1e-4);

    // Check keys are equal
    EXPECT_EQ(dedge_factor.front(), between_factor.front());
    EXPECT_EQ(dedge_factor.back(), between_factor.back());

    // Check Covariance
    EXPECT_EQ(0, e.covariance[0]);
    EXPECT_EQ(0, e.covariance[7]);
    EXPECT_EQ(0, e.covariance[14]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[21]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[28]);
    EXPECT_EQ(1.0 / 0.0, e.covariance[35]);
  }
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_kimera_pgmo");
  return RUN_ALL_TESTS();
}