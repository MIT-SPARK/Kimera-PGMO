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
#include <kimera_pgmo/utils/logging.h>
#include <ros/ros.h>

#include "kimera_pgmo_ros/conversion/mesh_conversion.h"
#include "kimera_pgmo_ros/kimera_pgmo.h"
#include "test_artifacts.h"

namespace kimera_pgmo {

using conversions::GtsamToRos;
using conversions::RosToGtsam;
using kimera_pgmo_msgs::KimeraPgmoMesh;
using kimera_pgmo_msgs::RequestMeshFactors;

namespace {
template <typename T, typename Ptr>
const T* cast_factor(const Ptr& ptr) {
  if (!ptr) {
    throw std::runtime_error("bad pointer");
  }

  auto cast = dynamic_cast<const T*>(ptr.get());
  if (!cast) {
    throw std::runtime_error("bad cast");
  }

  return cast;
}
}  // namespace

class KimeraDpgmoTest : public ::testing::Test {
 protected:
  KimeraDpgmoTest() : nh("~") {
    nh.setParam("frame_id", "world");
    nh.setParam("robot_id", 0);
    nh.setParam("run_mode", "DPGMO");
    nh.setParam("embed_trajectory_delta_t", 3.0);
    nh.setParam("num_interp_pts", 4);
    nh.setParam("interp_horizon", 10.0);
    nh.setParam("rpgo/odom_trans_threshold", 10.0);
    nh.setParam("rpgo/odom_rot_threshold", 10.0);
    nh.setParam("rpgo/pcm_trans_threshold", 10.0);
    nh.setParam("rpgo/pcm_rot_threshold", 10.0);
    nh.setParam("rpgo/gnc_alpha", 0);
    nh.setParam("add_initial_prior", true);
    nh.setParam("enable_sparsify", false);
    nh.setParam("covariance/odom", 0.000001);
    nh.setParam("covariance/loop_close", 0.0001);
    nh.setParam("covariance/prior", 0.00000001);
    nh.setParam("covariance/mesh_mesh", 0.001);
    nh.setParam("covariance/pose_mesh", 0.0001);
  }

  ~KimeraDpgmoTest() {}

  bool init() {
    const auto ret = pgmo_.initFromRos(nh);
    return ret;
  }

  void IncrementalPoseGraphCallback(const pose_graph_tools_msgs::PoseGraph& msg) {
    pgmo_.incrementalPoseGraphCallback(msg);
  }

  void FullMeshCallback(const KimeraPgmoMesh& mesh_msg) {
    pgmo_.fullMeshCallback(mesh_msg);
  }

  void IncrementalMeshGraphCallback(
      const pose_graph_tools_msgs::PoseGraph& mesh_graph_msg) {
    pgmo_.incrementalMeshGraphCallback(mesh_graph_msg);
  }

  void DpgmoCallback(const pose_graph_tools_msgs::PoseGraph& msg) {
    pgmo_.dpgmoCallback(msg);
  }

  bool RequestMeshEdgesCallback(RequestMeshFactors::Request& req,
                                RequestMeshFactors::Response& res) {
    return pgmo_.requestMeshEdgesCallback(req, res);
  }

  std::vector<gtsam::Pose3> getTrajectory() const { return pgmo_.trajectory_; }

  pcl::PolygonMesh getOptimizedMesh() const { return *(pgmo_.optimized_mesh_); }

  Path getOptimizedPath() const { return *(pgmo_.optimized_path_); }

  gtsam::NonlinearFactorGraph getConsistencyFactorsGtsam() const {
    return pgmo_.getDeformationGraphPtr()->getConsistencyFactors();
  }

  bool getConsistencyFactorsMsg(size_t robot_id,
                                pose_graph_tools::PoseGraph& pg_mesh,
                                size_t vertex_index_offset) const {
    return pgmo_.getConsistencyFactors(robot_id, pg_mesh, vertex_index_offset);
  }

  gtsam::Values getDpgmoValues() const { return pgmo_.getDpgmoValues(); }

  ros::NodeHandle nh;
  KimeraPgmo pgmo_;
};

TEST_F(KimeraDpgmoTest, fullMeshCallback) {
  ASSERT_TRUE(init());

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

  // Expect no distortion
  EXPECT_EQ(5u, optimized_vertices.points.size());
  EXPECT_EQ(2, optimized_vertices.points[0].x);
  EXPECT_EQ(2, optimized_vertices.points[0].y);
  EXPECT_EQ(2, optimized_vertices.points[0].z);

  EXPECT_EQ(2, optimized_vertices.points[4].x);
  EXPECT_EQ(2, optimized_vertices.points[4].y);
  EXPECT_EQ(3, optimized_vertices.points[4].z);
}

TEST_F(KimeraDpgmoTest, getConsistencyFactors) {
  ASSERT_TRUE(init());

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

  gtsam::NonlinearFactorGraph consistency_factors = getConsistencyFactorsGtsam();
  pose_graph_tools::PoseGraph consistency_edges_pg;

  // First test with no offset
  getConsistencyFactorsMsg(0, consistency_edges_pg, 0);

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
        *cast_factor<DeformationEdgeFactor>(consistency_factors[i]);
    // Create the between factor that is equivalent to the pose graph edge
    const auto& e = consistency_edges_pg.edges[i];
    gtsam::Key front, back;
    switch (e.type) {
      case pose_graph_tools::PoseGraphEdge::Type::MESH: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to), e.key_to);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::Type::POSE_MESH: {
        front = gtsam::Symbol(robot_id_to_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to), e.key_to);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::Type::MESH_POSE: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_prefix.at(e.robot_to), e.key_to);
        break;
      }
      default:
        FAIL();
    }
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(
        front, back, gtsam::Pose3(e.pose.matrix()), between_noise);

    // Create perturbed poses
    gtsam::Pose3 pose_front =
        dedge_factor.fromPose() *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0.01, 0, 0), gtsam::Point3(0, 0, 0.001));
    gtsam::Pose3 pose_back =
        gtsam::Pose3(gtsam::Rot3(), dedge_factor.toPoint()) *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0, 0, 0.01), gtsam::Point3(-0.001, 0, 0));

    // Check errors are equal
    gtsam::Vector betweenError = between_factor.evaluateError(pose_front, pose_back);

    gtsam::Vector dedgeError = dedge_factor.evaluateError(pose_front, pose_back);

    EXPECT_NEAR(-dedgeError(0), betweenError(3), 1e-4);
    EXPECT_NEAR(-dedgeError(1), betweenError(4), 1e-4);
    EXPECT_NEAR(-dedgeError(2), betweenError(5), 1e-4);

    // Check keys are equal
    EXPECT_EQ(dedge_factor.front(), between_factor.front());
    EXPECT_EQ(dedge_factor.back(), between_factor.back());

    // Check Covariance
    EXPECT_EQ(0, e.covariance(0, 0));
    EXPECT_EQ(0, e.covariance(1, 10));
    EXPECT_EQ(0, e.covariance(2, 2));
    EXPECT_EQ(1.0 / 0.0, e.covariance(3, 3));
    EXPECT_EQ(1.0 / 0.0, e.covariance(4, 4));
    EXPECT_EQ(1.0 / 0.0, e.covariance(5, 5));
  }

  // Check nodes
  EXPECT_EQ(10, consistency_edges_pg.nodes.size());
  EXPECT_EQ(0, consistency_edges_pg.nodes[0].robot_id);
  EXPECT_EQ(0, consistency_edges_pg.nodes[0].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                          gtsam::Pose3(consistency_edges_pg.nodes[0].pose.matrix())));
  EXPECT_EQ(0, consistency_edges_pg.nodes[9].robot_id);
  EXPECT_EQ(9, consistency_edges_pg.nodes[9].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 1)),
                          gtsam::Pose3(consistency_edges_pg.nodes[9].pose.matrix())));
}

TEST_F(KimeraDpgmoTest, RequestMeshEdgesCallback) {
  ASSERT_TRUE(init());

  // Test no reindex
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

  RequestMeshFactors::Request request;
  RequestMeshFactors::Response response;

  request.robot_id = 1;  // first test with wrong robot id

  EXPECT_FALSE(RequestMeshEdgesCallback(request, response));

  request.robot_id = 0;              // test with correct robot id
  request.reindex_vertices = false;  // reindex

  EXPECT_TRUE(RequestMeshEdgesCallback(request, response));
  gtsam::NonlinearFactorGraph consistency_factors = getConsistencyFactorsGtsam();
  std::vector<pose_graph_tools_msgs::PoseGraphEdge> consistency_edges =
      response.mesh_factors.edges;
  std::vector<pose_graph_tools_msgs::PoseGraphNode> consistency_nodes =
      response.mesh_factors.nodes;

  // Arbitrary noise
  gtsam::Vector6 precisions_6;
  precisions_6.head<3>().setConstant(0.0);
  precisions_6.tail<3>().setConstant(1e3);
  static const gtsam::SharedNoiseModel& between_noise =
      gtsam::noiseModel::Diagonal::Precisions(precisions_6);

  // Iterate and check edges
  for (size_t i = 0; i < consistency_factors.size(); i++) {
    // Compare the two
    DeformationEdgeFactor dedge_factor =
        *cast_factor<DeformationEdgeFactor>(consistency_factors[i]);
    // Create the between factor that is equivalent to the pose graph edge
    pose_graph_tools_msgs::PoseGraphEdge e = consistency_edges[i];
    gtsam::Key front, back;
    switch (e.type) {
      case pose_graph_tools_msgs::PoseGraphEdge::MESH: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to), e.key_to);
        break;
      }
      case pose_graph_tools_msgs::PoseGraphEdge::POSE_MESH: {
        front = gtsam::Symbol(robot_id_to_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to), e.key_to);
        break;
      }
      case pose_graph_tools_msgs::PoseGraphEdge::MESH_POSE: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_prefix.at(e.robot_to), e.key_to);
        break;
      }
      default:
        FAIL();
    }
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(
        front, back, RosToGtsam(e.pose), between_noise);

    // Create perturbed poses
    gtsam::Pose3 pose_front =
        dedge_factor.fromPose() *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0.01, 0, 0), gtsam::Point3(0, 0, 0.001));
    gtsam::Pose3 pose_back =
        gtsam::Pose3(gtsam::Rot3(), dedge_factor.toPoint()) *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0, 0, 0.01), gtsam::Point3(-0.001, 0, 0));

    // Check errors are equal
    gtsam::Vector betweenError = between_factor.evaluateError(pose_front, pose_back);

    gtsam::Vector dedgeError = dedge_factor.evaluateError(pose_front, pose_back);

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

  // Check nodes
  EXPECT_EQ(10, consistency_nodes.size());
  EXPECT_EQ(0, consistency_nodes[0].robot_id);
  EXPECT_EQ(0, consistency_nodes[0].key);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                                  RosToGtsam(consistency_nodes[0].pose)));
  EXPECT_EQ(0, consistency_nodes[9].robot_id);
  EXPECT_EQ(9, consistency_nodes[9].key);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 1)),
                                  RosToGtsam(consistency_nodes[9].pose)));
}

TEST_F(KimeraDpgmoTest, RequestMeshEdgesCallbackReindex) {
  ASSERT_TRUE(init());

  // Test reindex
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

  RequestMeshFactors::Request request;
  RequestMeshFactors::Response response;

  request.robot_id = 1;             // first test with wrong robot id
  request.reindex_vertices = true;  // do not reindex

  EXPECT_FALSE(RequestMeshEdgesCallback(request, response));

  request.robot_id = 0;  // test with correct robot id

  EXPECT_TRUE(RequestMeshEdgesCallback(request, response));
  gtsam::NonlinearFactorGraph consistency_factors = getConsistencyFactorsGtsam();
  std::vector<pose_graph_tools_msgs::PoseGraphEdge> consistency_edges =
      response.mesh_factors.edges;
  std::vector<pose_graph_tools_msgs::PoseGraphNode> consistency_nodes =
      response.mesh_factors.nodes;

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
        *cast_factor<DeformationEdgeFactor>(consistency_factors[i]);
    // Create the between factor that is equivalent to the pose graph edge
    pose_graph_tools_msgs::PoseGraphEdge e = consistency_edges[i];
    gtsam::Key front, back;
    switch (e.type) {
      case pose_graph_tools_msgs::PoseGraphEdge::MESH: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from - num_poses);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to),
                             e.key_to - num_poses);
        break;
      }
      case pose_graph_tools_msgs::PoseGraphEdge::POSE_MESH: {
        front = gtsam::Symbol(robot_id_to_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to),
                             e.key_to - num_poses);
        break;
      }
      case pose_graph_tools_msgs::PoseGraphEdge::MESH_POSE: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from - num_poses);
        back = gtsam::Symbol(robot_id_to_prefix.at(e.robot_to), e.key_to);
        break;
      }
      default:
        FAIL();
    }
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(
        front, back, RosToGtsam(e.pose), between_noise);

    // Create perturbed poses
    gtsam::Pose3 pose_front =
        dedge_factor.fromPose() *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0.01, 0, 0), gtsam::Point3(0, 0, 0.001));
    gtsam::Pose3 pose_back =
        gtsam::Pose3(gtsam::Rot3(), dedge_factor.toPoint()) *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0, 0, 0.01), gtsam::Point3(-0.001, 0, 0));

    // Check errors are equal
    gtsam::Vector betweenError = between_factor.evaluateError(pose_front, pose_back);

    gtsam::Vector dedgeError = dedge_factor.evaluateError(pose_front, pose_back);

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

  // Check nodes
  EXPECT_EQ(10, consistency_nodes.size());
  EXPECT_EQ(0, consistency_nodes[0].robot_id);
  EXPECT_EQ(num_poses, consistency_nodes[0].key);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                                  RosToGtsam(consistency_nodes[0].pose)));
  EXPECT_EQ(0, consistency_nodes[9].robot_id);
  EXPECT_EQ(num_poses + 9, consistency_nodes[9].key);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 1)),
                                  RosToGtsam(consistency_nodes[9].pose)));
}

TEST_F(KimeraDpgmoTest, CheckRobotIdRequestMeshEdgesCallback) {
  nh.setParam("robot_id", 2);
  ASSERT_TRUE(init());

  // Test reindex
  // Here we should test if the mesh is added to the deformation graph correctly
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  auto inc_graph = SingleOdomGraph(ros::Time(10.2), 2);
  IncrementalPoseGraphCallback(inc_graph);

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

  RequestMeshFactors::Request request;
  RequestMeshFactors::Response response;

  request.robot_id = 0;             // first test with wrong robot id
  request.reindex_vertices = true;  // reindex

  EXPECT_FALSE(RequestMeshEdgesCallback(request, response));

  request.robot_id = 2;  // test with correct robot id

  EXPECT_TRUE(RequestMeshEdgesCallback(request, response));
  gtsam::NonlinearFactorGraph consistency_factors = getConsistencyFactorsGtsam();
  std::vector<pose_graph_tools_msgs::PoseGraphEdge> consistency_edges =
      response.mesh_factors.edges;
  std::vector<pose_graph_tools_msgs::PoseGraphNode> consistency_nodes =
      response.mesh_factors.nodes;

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
        *cast_factor<DeformationEdgeFactor>(consistency_factors[i]);
    // Create the between factor that is equivalent to the pose graph edge
    pose_graph_tools_msgs::PoseGraphEdge e = consistency_edges[i];
    gtsam::Key front, back;
    switch (e.type) {
      case pose_graph_tools_msgs::PoseGraphEdge::MESH: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from - num_poses);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to),
                             e.key_to - num_poses);
        break;
      }
      case pose_graph_tools_msgs::PoseGraphEdge::POSE_MESH: {
        front = gtsam::Symbol(robot_id_to_prefix.at(e.robot_from), e.key_from);
        back = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_to),
                             e.key_to - num_poses);
        break;
      }
      case pose_graph_tools_msgs::PoseGraphEdge::MESH_POSE: {
        front = gtsam::Symbol(robot_id_to_vertex_prefix.at(e.robot_from),
                              e.key_from - num_poses);
        back = gtsam::Symbol(robot_id_to_prefix.at(e.robot_to), e.key_to);
        break;
      }
      default:
        FAIL();
    }
    gtsam::BetweenFactor<gtsam::Pose3> between_factor(
        front, back, RosToGtsam(e.pose), between_noise);

    // Create perturbed poses
    gtsam::Pose3 pose_front =
        dedge_factor.fromPose() *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0.01, 0, 0), gtsam::Point3(0, 0, 0.001));
    gtsam::Pose3 pose_back =
        gtsam::Pose3(gtsam::Rot3(), dedge_factor.toPoint()) *
        gtsam::Pose3(gtsam::Rot3::Rodrigues(0, 0, 0.01), gtsam::Point3(-0.001, 0, 0));

    // Check errors are equal
    gtsam::Vector betweenError = between_factor.evaluateError(pose_front, pose_back);

    gtsam::Vector dedgeError = dedge_factor.evaluateError(pose_front, pose_back);

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

  // Check nodes
  EXPECT_EQ(10, consistency_nodes.size());
  EXPECT_EQ(2, consistency_nodes[0].robot_id);
  EXPECT_EQ(num_poses, consistency_nodes[0].key);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                                  RosToGtsam(consistency_nodes[0].pose)));
  EXPECT_EQ(2, consistency_nodes[9].robot_id);
  EXPECT_EQ(num_poses + 9, consistency_nodes[9].key);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 1)),
                                  RosToGtsam(consistency_nodes[9].pose)));
}

TEST_F(KimeraDpgmoTest, dpgmoCallbackValues) {
  ASSERT_TRUE(init());

  // Test reindex
  // Here we should test if the mesh is added to the deformation graph correctly
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  auto inc_graph_1 = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph_1);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  auto mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // load second incremental pose graph
  auto inc_graph_2 = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph_2);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  RequestMeshFactors::Request request;
  RequestMeshFactors::Response response;

  request.robot_id = 0;             // first test with wrong robot id
  request.reindex_vertices = true;  // do not reindex
  RequestMeshEdgesCallback(request, response);

  // Create DPGMO message
  pose_graph_tools_msgs::PoseGraph dpgmo_msg;
  for (const auto& node : inc_graph_1.nodes) {
    dpgmo_msg.nodes.push_back(node);
  }

  dpgmo_msg.nodes.push_back(inc_graph_2.nodes[1]);

  for (const auto& node : response.mesh_factors.nodes) {
    dpgmo_msg.nodes.push_back(node);
  }

  DpgmoCallback(dpgmo_msg);

  gtsam::Values dpgmo_values = getDpgmoValues();
  size_t num_poses = getTrajectory().size();
  EXPECT_EQ(num_poses + 10, dpgmo_values.size());
  for (const auto& node : inc_graph_1.nodes) {
    gtsam::Key key = gtsam::Symbol(robot_id_to_prefix.at(node.robot_id), node.key);
    EXPECT_TRUE(
        gtsam::assert_equal(RosToGtsam(node.pose), dpgmo_values.at<gtsam::Pose3>(key)));
  }

  pose_graph_tools_msgs::PoseGraphNode node = inc_graph_2.nodes[1];
  gtsam::Key key = gtsam::Symbol(robot_id_to_prefix.at(node.robot_id), node.key);
  EXPECT_TRUE(
      gtsam::assert_equal(RosToGtsam(node.pose), dpgmo_values.at<gtsam::Pose3>(key)));

  for (auto node : response.mesh_factors.nodes) {
    gtsam::Key key = gtsam::Symbol(robot_id_to_vertex_prefix.at(node.robot_id),
                                   node.key - num_poses);
    EXPECT_TRUE(
        gtsam::assert_equal(RosToGtsam(node.pose), dpgmo_values.at<gtsam::Pose3>(key)));
  }
}

TEST_F(KimeraDpgmoTest, dpgmoCallbackDeform) {
  ASSERT_TRUE(init());

  // Test reindex
  // Here we should test if the mesh is added to the deformation graph correctly
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  auto inc_graph_1 = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph_1);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  auto mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // load second incremental pose graph
  auto inc_graph_2 = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph_2);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  RequestMeshFactors::Request request;
  RequestMeshFactors::Response response;

  request.robot_id = 0;             // first test with wrong robot id
  request.reindex_vertices = true;  // do not reindex
  RequestMeshEdgesCallback(request, response);

  // Create DPGMO message
  pose_graph_tools_msgs::PoseGraph dpgmo_msg;
  for (auto node : inc_graph_1.nodes) {
    pose_graph_tools_msgs::PoseGraphNode optimized_node = node;
    optimized_node.pose.position.x += 1;
    dpgmo_msg.nodes.push_back(optimized_node);
  }

  pose_graph_tools_msgs::PoseGraphNode optimized_node = inc_graph_2.nodes[1];
  optimized_node.pose.position.x += 1;
  dpgmo_msg.nodes.push_back(optimized_node);

  for (auto node : response.mesh_factors.nodes) {
    pose_graph_tools_msgs::PoseGraphNode optimized_node = node;
    optimized_node.pose.position.x += 1;
    dpgmo_msg.nodes.push_back(optimized_node);
  }

  DpgmoCallback(dpgmo_msg);

  pcl::PolygonMesh full_mesh = createMesh(1, 0, 0);
  std::vector<Timestamp> full_vertex_stamps;
  for (size_t i = 0; i < full_mesh.cloud.width * full_mesh.cloud.height; i++) {
    full_vertex_stamps.push_back(stampFromSec(13.0));
  }

  auto full_mesh_msg = conversions::toMsg(0, full_mesh, full_vertex_stamps, "world");
  ASSERT_TRUE(full_mesh_msg);
  FullMeshCallback(*full_mesh_msg);

  pcl::PolygonMesh opt_mesh = getOptimizedMesh();
  std::vector<gtsam::Pose3> opt_path = getOptimizedPath();

  // Check optimized path
  std::vector<gtsam::Pose3> init_path = getTrajectory();
  for (size_t i = 0; i < init_path.size(); i++) {
    gtsam::Rot3 init_rot = init_path[i].rotation();
    gtsam::Point3 init_pos = init_path[i].translation();
    gtsam::Pose3 expected_pose = gtsam::Pose3(
        init_rot, gtsam::Point3(init_pos.x() + 1.0, init_pos.y(), init_pos.z()));
    EXPECT_TRUE(gtsam::assert_equal(expected_pose, opt_path.at(i)));
  }

  // Check optimized vertices
  pcl::PointCloud<pcl::PointXYZRGBA> optimized_vertices;
  pcl::fromPCLPointCloud2(opt_mesh.cloud, optimized_vertices);

  EXPECT_EQ(5u, optimized_vertices.points.size());
  EXPECT_EQ(2, optimized_vertices.points[0].x);
  EXPECT_EQ(0, optimized_vertices.points[0].y);
  EXPECT_EQ(0, optimized_vertices.points[0].z);

  EXPECT_EQ(2, optimized_vertices.points[2].x);
  EXPECT_EQ(1, optimized_vertices.points[2].y);
  EXPECT_EQ(0, optimized_vertices.points[2].z);

  EXPECT_EQ(2, optimized_vertices.points[4].x);
  EXPECT_EQ(0, optimized_vertices.points[4].y);
  EXPECT_EQ(1, optimized_vertices.points[4].z);
}

}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  logging::Logger::addSink("cout",
                           std::make_shared<logging::CoutSink>(logging::Level::FATAL));
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_ros-test_kimera_dpgmo");
  return RUN_ALL_TESTS();
}
