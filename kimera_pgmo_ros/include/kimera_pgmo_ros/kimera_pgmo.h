/**
 * @file   kimera_pgmo.h
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#pragma once

#include <kimera_pgmo/kimera_pgmo_interface.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <pose_graph_tools_ros/conversions.h>
#include <tf2_ros/transform_broadcaster.h>

#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include <kimera_pgmo_msgs/msg/absolute_pose_stamped.hpp>
#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <kimera_pgmo_msgs/srv/load_graph_mesh.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace kimera_pgmo {

class KimeraPgmo : public KimeraPgmoInterface, public rclcpp::Node {
  friend class KimeraPgmoTest;
  friend class KimeraDpgmoTest;

 public:
  using MeshMsg = kimera_pgmo_msgs::msg::Mesh;
  using OdomMsg = nav_msgs::msg::Odometry;
  using PathMsg = nav_msgs::msg::Path;
  using MarkerMsg = visualization_msgs::msg::Marker;
  using LoadGraphSrv = kimera_pgmo_msgs::srv::LoadGraphMesh;

  struct Config : KimeraPgmoConfig {
    std::string frame_id;
    int robot_id = 0;
  };

  explicit KimeraPgmo(const rclcpp::NodeOptions& options);

  ~KimeraPgmo();

  //! Get the current robot id
  int getRobotId() const { return config_.robot_id; }

  //! Get the current robot prefix
  char getRobotPrefix() const { return robot_id_to_prefix.at(config_.robot_id); }

  //! Get the timestamps for the robot
  std::vector<Timestamp> getRobotTimestamps() const { return timestamps_; }

 protected:
  //! Publish mesh
  bool publishMesh() const;

  //! Publish trajectory
  bool publishPath(const std_msgs::msg::Header& header,
                   rclcpp::Publisher<nav_msgs::msg::Path>& publisher) const;

  //! Start the thread doing the mesh graph / pose graph / path subscription
  void startGraphProcess();

  //! Start the thread doing the full mesh subscription */
  void startMeshProcess();

  //! Publish the optimized mesh (stored after deformation)
  bool publishOptimizedMesh() const;

  //! Publish optimized trajectory
  bool publishOptimizedPath() const;

  //! Publish the transform for each robot id based on the latest node in pose graph
  void publishTransforms();

  //! Publish markers for deformation graph
  void visualizeDeformationGraph() const;

  //! Handle updates to the pose graph
  void incrementalPoseGraphCallback(const pose_graph_tools::PoseGraph& msg);

  //! Handle updates to the full background mesh
  void fullMeshCallback(const kimera_pgmo_msgs::msg::Mesh& msg);

  //! Handle updates to the deformation graph control points
  void incrementalMeshGraphCallback(const pose_graph_tools::PoseGraph& msg);

  //! Update priors on trajectory from an external optimizer
  void optimizedPathCallback(const nav_msgs::msg::Path& msg);

  //! Saves mesh as a ply file.
  void saveMeshCallback(const std_srvs::srv::Empty::Request::SharedPtr&,
                        std_srvs::srv::Empty::Response::SharedPtr);

  //! Saves all the trajectories of all robots to csv files.
  void saveTrajectoryCallback(const std_srvs::srv::Empty::Request::SharedPtr&,
                              std_srvs::srv::Empty::Response::SharedPtr);

  //! Saves the deformation graph to a custom dgrf file.
  void saveGraphCallback(const std_srvs::srv::Empty::Request::SharedPtr&,
                         std_srvs::srv::Empty::Response::SharedPtr);

  //! Loads a deformation graph and associated mesh.
  void loadGraphMeshCallback(const LoadGraphSrv::Request::SharedPtr& req,
                             LoadGraphSrv::Response::SharedPtr rep);

  //! Clear and reset the deformation graph.
  void resetGraphCallback(const std_srvs::srv::Empty::Request::SharedPtr&,
                          std_srvs::srv::Empty::Response::SharedPtr);

 protected:
  Config config_;

  rclcpp::Time last_mesh_stamp_;

  // Publishers
  rclcpp::Publisher<MeshMsg>::SharedPtr optimized_mesh_pub_;
  rclcpp::Publisher<PathMsg>::SharedPtr optimized_path_pub_;
  rclcpp::Publisher<OdomMsg>::SharedPtr optimized_odom_pub_;
  pose_graph_tools::PoseGraphPublisher pose_graph_pub_;
  rclcpp::Publisher<MarkerMsg>::SharedPtr viz_mesh_mesh_edges_pub_;
  rclcpp::Publisher<MarkerMsg>::SharedPtr viz_pose_mesh_edges_pub_;

  // Transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;

  // Subscribers
  rclcpp::Subscription<MeshMsg>::SharedPtr full_mesh_sub_;
  pose_graph_tools::PoseGraphSubscription pose_graph_incremental_sub_;
  pose_graph_tools::PoseGraphSubscription incremental_mesh_graph_sub_;
  rclcpp::Subscription<PathMsg>::SharedPtr path_callback_sub_;

  // Service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_mesh_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_traj_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_graph_srv_;
  rclcpp::Service<kimera_pgmo_msgs::srv::LoadGraphMesh>::SharedPtr load_graph_mesh_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

  // Trajectory
  std::vector<gtsam::Pose3> trajectory_;
  std::vector<size_t> unconnected_mesh_indices_;
  std::vector<uint64_t> unconnected_mesh_index_stamps_;
  std::queue<size_t> dpgmo_num_poses_last_req_;

  std::unique_ptr<std::thread> graph_thread_;
  std::unique_ptr<std::thread> mesh_thread_;
  std::mutex interface_mutex_;

  // Time callback spin time
  int inc_mesh_cb_time_;
  int full_mesh_cb_time_;
  int pg_cb_time_;
  int path_cb_time_;
};

void declare_config(KimeraPgmo::Config& config);

}  // namespace kimera_pgmo
