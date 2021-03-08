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

#include "kimera_pgmo/KimeraPgmoInterface.h"

namespace kimera_pgmo {

class KimeraPgmoInterfaceTest : public ::testing::Test {
 protected:
  KimeraPgmoInterfaceTest() {
    system("rosparam set compression_time_horizon 10.0");
    system("rosparam set run_mode 0");
    system("rosparam set use_msg_time true");
    system("rosparam set embed_trajectory_delta_t 3.0");
    system("rosparam set rpgo/translation_threshold 10.0");
    system("rosparam set rpgo/rotation_threshold 10.0");
    ros::NodeHandle nh;
    compression_.reset(new OctreeCompression(0.5));
    pgmo_.loadParameters(nh);
  }
  ~KimeraPgmoInterfaceTest() {}

  inline bool GetConsistencyFactors(
      const size_t& robot_id,
      std::vector<pose_graph_tools::PoseGraphEdge>* edges,
      const size_t& vertex_index_offset) {
    pgmo_.getConsistencyFactors(robot_id, edges, vertex_index_offset);
  }

  inline void ProcessIncrementalPoseGraph(
      const pose_graph_tools::PoseGraph::ConstPtr& msg,
      std::vector<gtsam::Pose3>* initial_trajectory,
      std::queue<size_t>* unconnected_nodes,
      std::vector<ros::Time>* node_timestamps,
      bool single_robot) {
    pgmo_.processIncrementalPoseGraph(msg,
                                      initial_trajectory,
                                      unconnected_nodes,
                                      node_timestamps,
                                      single_robot);
  }

  inline void ProcessIncrementalMesh(
      const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg,
      const OctreeCompressionPtr compressor,
      const std::vector<ros::Time>& node_timestamps,
      std::queue<size_t>* unconnected_nodes,
      bool single_robot) {
    pgmo_.processIncrementalMesh(
        mesh_msg, compressor, node_timestamps, unconnected_nodes, single_robot);
  }

  inline void ProcessOptimizedPath(const nav_msgs::Path::ConstPtr& path_msg,
                                   const size_t& robot_id = 0) {
    pgmo_.processOptimizedPath(path_msg, robot_id);
  }

  inline DeformationGraph GetDeformationGraph() {
    return pgmo_.deformation_graph_;
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

  KimeraPgmoInterface pgmo_;
  OctreeCompressionPtr compression_;
};

TEST_F(KimeraPgmoInterfaceTest, initialize) {
  DeformationGraph d_graph = GetDeformationGraph();
  EXPECT_EQ(0, d_graph.getNumVertices());
  EXPECT_EQ(0, compression_->getNumVertices());
}
}  // namespace kimera_pgmo

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "kimera_pgmo_test_kimera_pgmo");
  return RUN_ALL_TESTS();
}
