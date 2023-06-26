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
#include "test_artifacts.h"

namespace kimera_pgmo {

class KimeraDpgmoTest : public ::testing::Test {
 protected:
  KimeraDpgmoTest() {
    system("rosparam set frame_id world");
    system("rosparam set robot_id 0");
    system("rosparam set run_mode 2");
    system("rosparam set embed_trajectory_delta_t 3.0");
    system("rosparam set num_interp_pts 4");
    system("rosparam set interp_horizon 10.0");
    system("rosparam set rpgo/odom_trans_threshold 10.0");
    system("rosparam set rpgo/odom_rot_threshold 10.0");
    system("rosparam set rpgo/pcm_trans_threshold 10.0");
    system("rosparam set rpgo/pcm_rot_threshold 10.0");
    system("rosparam set rpgo/gnc_alpha 0");
    system("rosparam set add_initial_prior true");
    system("rosparam set enable_sparsify false");
    system("rosparam set covariance/odom 0.000001");
    system("rosparam set covariance/loop_close 0.0001");
    system("rosparam set covariance/prior 0.00000001");
    system("rosparam set covariance/mesh_mesh 0.001");
    system("rosparam set covariance/pose_mesh 0.0001");
  }
  ~KimeraDpgmoTest() {}

  void IncrementalPoseGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& msg) {
    pgmo_.incrementalPoseGraphCallback(msg);
  }

  void FullMeshCallback(const KimeraPgmoMesh::ConstPtr& mesh_msg) {
    pgmo_.fullMeshCallback(mesh_msg);
  }

  void IncrementalMeshGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& mesh_graph_msg) {
    pgmo_.incrementalMeshGraphCallback(mesh_graph_msg);
  }

  void DpgmoCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg) {
    pgmo_.dpgmoCallback(msg);
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

  inline Path getOptimizedPath() const { return *(pgmo_.optimized_path_); }

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

  inline gtsam::Values getDpgmoValues() const { return pgmo_.getDpgmoValues(); }

  KimeraPgmo pgmo_;
};

TEST_F(KimeraDpgmoTest, initialize) {
  ros::NodeHandle nh;
  bool init = pgmo_.initialize(nh);

  ASSERT_TRUE(init);
}

TEST_F(KimeraDpgmoTest, fullMeshCallback) {
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
  std::vector<ros::Time> full_vertex_stamps;
  for (size_t i = 0; i < full_mesh.cloud.width * full_mesh.cloud.height; i++) {
    full_vertex_stamps.push_back(ros::Time(13.0));
  }
  KimeraPgmoMesh::Ptr full_mesh_msg(new KimeraPgmoMesh);
  *full_mesh_msg =
      PolygonMeshToPgmoMeshMsg(0, full_mesh, full_vertex_stamps, "world");
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

  // Expect no distortion
  EXPECT_EQ(size_t(5), optimized_vertices.points.size());
  EXPECT_EQ(2, optimized_vertices.points[0].x);
  EXPECT_EQ(2, optimized_vertices.points[0].y);
  EXPECT_EQ(2, optimized_vertices.points[0].z);

  EXPECT_EQ(2, optimized_vertices.points[4].x);
  EXPECT_EQ(2, optimized_vertices.points[4].y);
  EXPECT_EQ(3, optimized_vertices.points[4].z);
}

TEST_F(KimeraDpgmoTest, getConsistencyFactors) {
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

  gtsam::NonlinearFactorGraph consistency_factors =
      getConsistencyFactorsGtsam();
  pose_graph_tools::PoseGraph consistency_edges_pg;

  // First test with no offset
  getConsistencyFactorsMsg(0, &consistency_edges_pg, 0);

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
    pose_graph_tools::PoseGraphEdge e = consistency_edges_pg.edges[i];
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

  // Check nodes
  EXPECT_EQ(10, consistency_edges_pg.nodes.size());
  EXPECT_EQ(0, consistency_edges_pg.nodes[0].robot_id);
  EXPECT_EQ(0, consistency_edges_pg.nodes[0].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                          RosToGtsam(consistency_edges_pg.nodes[0].pose)));
  EXPECT_EQ(0, consistency_edges_pg.nodes[9].robot_id);
  EXPECT_EQ(9, consistency_edges_pg.nodes[9].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 1)),
                          RosToGtsam(consistency_edges_pg.nodes[9].pose)));
}

TEST_F(KimeraDpgmoTest, RequestMeshEdgesCallback) {
  // Test no reindex
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

  kimera_pgmo::RequestMeshFactors::Request request;
  kimera_pgmo::RequestMeshFactors::Response response;

  request.robot_id = 1;  // first test with wrong robot id

  EXPECT_FALSE(RequestMeshEdgesCallback(request, response));

  request.robot_id = 0;              // test with correct robot id
  request.reindex_vertices = false;  // reindex

  EXPECT_TRUE(RequestMeshEdgesCallback(request, response));
  gtsam::NonlinearFactorGraph consistency_factors =
      getConsistencyFactorsGtsam();
  std::vector<pose_graph_tools::PoseGraphEdge> consistency_edges =
      response.mesh_factors.edges;
  std::vector<pose_graph_tools::PoseGraphNode> consistency_nodes =
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

  // Check nodes
  EXPECT_EQ(10, consistency_nodes.size());
  EXPECT_EQ(0, consistency_nodes[0].robot_id);
  EXPECT_EQ(0, consistency_nodes[0].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                          RosToGtsam(consistency_nodes[0].pose)));
  EXPECT_EQ(0, consistency_nodes[9].robot_id);
  EXPECT_EQ(9, consistency_nodes[9].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 1)),
                          RosToGtsam(consistency_nodes[9].pose)));
}

TEST_F(KimeraDpgmoTest, RequestMeshEdgesCallbackReindex) {
  // Test reindex
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
      response.mesh_factors.edges;
  std::vector<pose_graph_tools::PoseGraphNode> consistency_nodes =
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

  // Check nodes
  EXPECT_EQ(10, consistency_nodes.size());
  EXPECT_EQ(0, consistency_nodes[0].robot_id);
  EXPECT_EQ(num_poses, consistency_nodes[0].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                          RosToGtsam(consistency_nodes[0].pose)));
  EXPECT_EQ(0, consistency_nodes[9].robot_id);
  EXPECT_EQ(num_poses + 9, consistency_nodes[9].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 1)),
                          RosToGtsam(consistency_nodes[9].pose)));
}

TEST_F(KimeraDpgmoTest, CheckRobotIdRequestMeshEdgesCallback) {
  // Test reindex
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  EXPECT_EQ(system("rosparam set robot_id 2"), 0);
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph(new pose_graph_tools::PoseGraph);
  *inc_graph = SingleOdomGraph(ros::Time(10.2), 2);
  IncrementalPoseGraphCallback(inc_graph);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(
      new pose_graph_tools::PoseGraph);
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

  kimera_pgmo::RequestMeshFactors::Request request;
  kimera_pgmo::RequestMeshFactors::Response response;

  request.robot_id = 0;             // first test with wrong robot id
  request.reindex_vertices = true;  // reindex

  EXPECT_FALSE(RequestMeshEdgesCallback(request, response));

  request.robot_id = 2;  // test with correct robot id

  EXPECT_TRUE(RequestMeshEdgesCallback(request, response));
  gtsam::NonlinearFactorGraph consistency_factors =
      getConsistencyFactorsGtsam();
  std::vector<pose_graph_tools::PoseGraphEdge> consistency_edges =
      response.mesh_factors.edges;
  std::vector<pose_graph_tools::PoseGraphNode> consistency_nodes =
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

  // Check nodes
  EXPECT_EQ(10, consistency_nodes.size());
  EXPECT_EQ(2, consistency_nodes[0].robot_id);
  EXPECT_EQ(num_poses, consistency_nodes[0].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(0, 0, 0)),
                          RosToGtsam(consistency_nodes[0].pose)));
  EXPECT_EQ(2, consistency_nodes[9].robot_id);
  EXPECT_EQ(num_poses + 9, consistency_nodes[9].key);
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(), gtsam::Point3(2, 0, 1)),
                          RosToGtsam(consistency_nodes[9].pose)));
}

TEST_F(KimeraDpgmoTest, dpgmoCallbackValues) {
  // Test reindex
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph_1(new pose_graph_tools::PoseGraph);
  *inc_graph_1 = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph_1);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(
      new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // load second incremental pose graph
  pose_graph_tools::PoseGraph::Ptr inc_graph_2(new pose_graph_tools::PoseGraph);
  *inc_graph_2 = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph_2);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  kimera_pgmo::RequestMeshFactors::Request request;
  kimera_pgmo::RequestMeshFactors::Response response;

  request.robot_id = 0;             // first test with wrong robot id
  request.reindex_vertices = true;  // do not reindex
  RequestMeshEdgesCallback(request, response);

  // Create DPGMO message
  pose_graph_tools::PoseGraph::Ptr dpgmo_msg(new pose_graph_tools::PoseGraph);
  for (auto node : inc_graph_1->nodes) {
    pose_graph_tools::PoseGraphNode optimized_node = node;
    dpgmo_msg->nodes.push_back(optimized_node);
  }
  pose_graph_tools::PoseGraphNode optimized_node = inc_graph_2->nodes[1];
  dpgmo_msg->nodes.push_back(optimized_node);

  for (auto node : response.mesh_factors.nodes) {
    pose_graph_tools::PoseGraphNode optimized_node = node;
    dpgmo_msg->nodes.push_back(optimized_node);
  }

  DpgmoCallback(dpgmo_msg);

  gtsam::Values dpgmo_values = getDpgmoValues();
  size_t num_poses = getTrajectory().size();
  EXPECT_EQ(num_poses + 10, dpgmo_values.size());
  for (auto node : inc_graph_1->nodes) {
    gtsam::Key key =
        gtsam::Symbol(robot_id_to_prefix.at(node.robot_id), node.key);
    EXPECT_TRUE(gtsam::assert_equal(RosToGtsam(node.pose),
                                    dpgmo_values.at<gtsam::Pose3>(key)));
  }
  pose_graph_tools::PoseGraphNode node = inc_graph_2->nodes[1];
  gtsam::Key key =
      gtsam::Symbol(robot_id_to_prefix.at(node.robot_id), node.key);
  EXPECT_TRUE(gtsam::assert_equal(RosToGtsam(node.pose),
                                  dpgmo_values.at<gtsam::Pose3>(key)));

  for (auto node : response.mesh_factors.nodes) {
    gtsam::Key key = gtsam::Symbol(robot_id_to_vertex_prefix.at(node.robot_id),
                                   node.key - num_poses);
    EXPECT_TRUE(gtsam::assert_equal(RosToGtsam(node.pose),
                                    dpgmo_values.at<gtsam::Pose3>(key)));
  }
}

TEST_F(KimeraDpgmoTest, dpgmoCallbackDeform) {
  // Test reindex
  // Here we should test if the mesh is added to the deformation graph correctly
  ros::NodeHandle nh;
  pgmo_.initialize(nh);
  OctreeCompressionPtr compression(new OctreeCompression(0.5));
  Graph graph_struct;

  // Check callback
  pose_graph_tools::PoseGraph::Ptr inc_graph_1(new pose_graph_tools::PoseGraph);
  *inc_graph_1 = SingleOdomGraph(ros::Time(10.2), 0);
  IncrementalPoseGraphCallback(inc_graph_1);

  // Add mesh
  pcl::PolygonMesh mesh1 = createMesh(0, 0, 0);
  pose_graph_tools::PoseGraph::Ptr mesh_graph_msg(
      new pose_graph_tools::PoseGraph);
  *mesh_graph_msg =
      processMeshToGraph(mesh1, 0, ros::Time(12.5), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  // load second incremental pose graph
  pose_graph_tools::PoseGraph::Ptr inc_graph_2(new pose_graph_tools::PoseGraph);
  *inc_graph_2 = OdomLoopclosureGraph(ros::Time(12.8), 0);
  IncrementalPoseGraphCallback(inc_graph_2);

  // Add mesh
  pcl::PolygonMesh mesh2 = createMesh(2, 0, 0);
  *mesh_graph_msg =
      processMeshToGraph(mesh2, 0, ros::Time(13.0), compression, &graph_struct);
  IncrementalMeshGraphCallback(mesh_graph_msg);

  kimera_pgmo::RequestMeshFactors::Request request;
  kimera_pgmo::RequestMeshFactors::Response response;

  request.robot_id = 0;             // first test with wrong robot id
  request.reindex_vertices = true;  // do not reindex
  RequestMeshEdgesCallback(request, response);

  // Create DPGMO message
  pose_graph_tools::PoseGraph::Ptr dpgmo_msg(new pose_graph_tools::PoseGraph);
  for (auto node : inc_graph_1->nodes) {
    pose_graph_tools::PoseGraphNode optimized_node = node;
    optimized_node.pose.position.x += 1;
    dpgmo_msg->nodes.push_back(optimized_node);
  }
  pose_graph_tools::PoseGraphNode optimized_node = inc_graph_2->nodes[1];
  optimized_node.pose.position.x += 1;
  dpgmo_msg->nodes.push_back(optimized_node);

  for (auto node : response.mesh_factors.nodes) {
    pose_graph_tools::PoseGraphNode optimized_node = node;
    optimized_node.pose.position.x += 1;
    dpgmo_msg->nodes.push_back(optimized_node);
  }

  DpgmoCallback(dpgmo_msg);

  pcl::PolygonMesh full_mesh = createMesh(1, 0, 0);
  std::vector<ros::Time> full_vertex_stamps;
  for (size_t i = 0; i < full_mesh.cloud.width * full_mesh.cloud.height; i++) {
    full_vertex_stamps.push_back(ros::Time(13.0));
  }
  KimeraPgmoMesh::Ptr full_mesh_msg(new KimeraPgmoMesh);
  *full_mesh_msg =
      PolygonMeshToPgmoMeshMsg(0, full_mesh, full_vertex_stamps, "world");
  FullMeshCallback(full_mesh_msg);

  pcl::PolygonMesh opt_mesh = getOptimizedMesh();
  std::vector<gtsam::Pose3> opt_path = getOptimizedPath();

  // Check optimized path
  std::vector<gtsam::Pose3> init_path = getTrajectory();
  for (size_t i = 0; i < init_path.size(); i++) {
    gtsam::Rot3 init_rot = init_path[i].rotation();
    gtsam::Point3 init_pos = init_path[i].translation();
    gtsam::Pose3 expected_pose = gtsam::Pose3(
        init_rot,
        gtsam::Point3(init_pos.x() + 1.0, init_pos.y(), init_pos.z()));
    EXPECT_TRUE(gtsam::assert_equal(expected_pose, opt_path.at(i)));
  }

  // Check optimized vertices
  pcl::PointCloud<pcl::PointXYZRGBA> optimized_vertices;
  pcl::fromPCLPointCloud2(opt_mesh.cloud, optimized_vertices);

  EXPECT_EQ(size_t(5), optimized_vertices.points.size());
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
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_kimera_pgmo");
  return RUN_ALL_TESTS();
}
