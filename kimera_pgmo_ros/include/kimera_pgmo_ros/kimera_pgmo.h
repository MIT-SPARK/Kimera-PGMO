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
    // Save output
    std::string output_prefix;
    // Log output to output_prefix_ folder
    bool log_output;
  };

  /*! \brief Constructor for Kimera Pgmo class. Which subscribes to the
   * incremental mesh and pose graph to create the deformation graph and also
   * the full mesh to perform distortions and publish the optimzed distored mesh
   * and trajectory
   */
  explicit KimeraPgmo(const rclcpp::NodeOptions& options);

  ~KimeraPgmo();

  //! \brief Get a pointer to the optimized mesh
  pcl::PolygonMesh::ConstPtr getOptimizedMeshPtr() const { return optimized_mesh_; }

  //! \brief Get the current robot id
  int getRobotId() const { return config_.robot_id; }

  //! \brief Get the current robot prefix
  char getRobotPrefix() const { return robot_id_to_prefix.at(config_.robot_id); }

  //! \brief Get the timestamps for the robot
  std::vector<Timestamp> getRobotTimestamps() const { return timestamps_; }

 protected:
  /*! \brief Publish mesh
   * - mesh: mesh to publish
   * - header: header for the published message
   * - publisher: associated ros publisher
   */
  bool publishMesh(const pcl::PolygonMesh& mesh) const;

  /*! \brief Publish trajectory
   * - path: path to publish
   * - header: header for published message
   * - publisher: associated publisher
   */
  bool publishPath(const Path& path,
                   const std_msgs::msg::Header& header,
                   rclcpp::Publisher<nav_msgs::msg::Path>& publisher) const;

  /*! \brief Start the thread doing the mesh graph / pose graph / path * subscription */
  void startGraphProcess();

  /*! \brief Start the thread doing the full mesh subscription */
  void startMeshProcess();

  /*! \brief Publish the optimized mesh (stored after deformation)
   */
  bool publishOptimizedMesh() const;

  /*! \brief Publish optimized trajectory (Currently unused, as trajectory can
   * be visualized with published pose graph)
   *  - robot_id: the robot for which the trajectory is to be published
   */
  bool publishOptimizedPath() const;

  /*! \brief Publish the transform for each robot id based on the latest node in
   * pose graph
   */
  void publishTransforms();

  /*! \brief Publish markers for deformation graph */
  void visualizeDeformationGraph() const;

  /*! \brief Recieves latest edges in the pose graph and add to deformation
   * graph. Also place the received node in a queue to connect them to the
   * incremental mesh when that comes in
   *  - msg: new Pose Graph message consisting of the newest pose graph edges
   */
  void incrementalPoseGraphCallback(const pose_graph_tools::PoseGraph& msg);

  /*! \brief Subscribes to the full mesh and deform it based on the deformation
   * graph. Then publish the deformed mesh, and also the optimized pose graph
   *  - mesh_msg: the full unoptimized mesh in mesh_msgs TriangleMeshStamped
   * format
   */
  void fullMeshCallback(const kimera_pgmo_msgs::msg::Mesh& msg);

  /*! \brief Subscribes to the mesh factors from MeshFrontend, which
   * corresponds to the latest simplified partial mesh from Voxblox or
   * Kimera-Semantics. We add to the deformation graph and also connect the
   * nodes stored in the waiting queue to the vertices of the sampled mesh,
   *  - mesh_graph_msg: mesh factors to add to deformation graph and mesh nodes.
   */
  void incrementalMeshGraphCallback(const pose_graph_tools::PoseGraph& msg);

  /*! \brief Subscribes to an optimized trajectory. The path should correspond
   * to the nodes of the pose graph received in the
   * incrementalPoseGraphCallback. Note that this should only be used in the
   * single robot pose graph case.
   *  - mesh_msg: partial mesh in mesh_msgs TriangleMeshStamped format
   */
  void optimizedPathCallback(const nav_msgs::msg::Path& msg);

  /*! \brief Saves mesh as a ply file. Triggers through a rosservice call
   * and saves to file [output_prefix_]/mesh_pgmo.ply
   */
  void saveMeshCallback(const std_srvs::srv::Empty::Request::SharedPtr&,
                        std_srvs::srv::Empty::Response::SharedPtr);

  /*! \brief Saves all the trajectories of all robots to csv files. Triggers
   * through a rosservice call and saves to file [output_prefix_]/traj_pgmo.csv
   */
  void saveTrajectoryCallback(const std_srvs::srv::Empty::Request::SharedPtr&,
                              std_srvs::srv::Empty::Response::SharedPtr);

  /*! \brief Saves the deformation graph to a custom dgrf file. Triggers
   * through a rosservice call and saves to file [output_prefix_]/pgmo.dgrf
   */
  void saveGraphCallback(const std_srvs::srv::Empty::Request::SharedPtr&,
                         std_srvs::srv::Empty::Response::SharedPtr);

  /*! \brief Loads a deformation graph and associated mesh.
   */
  void loadGraphMeshCallback(const LoadGraphSrv::Request::SharedPtr& req,
                             LoadGraphSrv::Response::SharedPtr rep);

  /*! \brief log the run-time stats such as pose graph size, mesh size, and run
   * time
   */
  void logStats(const std::string& filename) const;

  /*! \brief Clear and reset the deformation graph.
   */
  void resetGraphCallback(const std_srvs::srv::Empty::Request::SharedPtr&,
                          std_srvs::srv::Empty::Response::SharedPtr);

 protected:
  Config config_;
  // optimized mesh for each robot
  pcl::PolygonMesh::Ptr optimized_mesh_;
  std::vector<Timestamp> mesh_vertex_stamps_;

  PathPtr optimized_path_;
  rclcpp::Time last_mesh_stamp_;

  // Publishers
  rclcpp::Publisher<MeshMsg>::SharedPtr optimized_mesh_pub_;
  rclcpp::Publisher<PathMsg>::SharedPtr optimized_path_pub_;  // Unused for now (TODO)
  rclcpp::Publisher<OdomMsg>::SharedPtr optimized_odom_pub_;  // Unused for now (TODO)
  pose_graph_tools::PoseGraphPublisher pose_graph_pub_;
  rclcpp::Publisher<MarkerMsg>::SharedPtr viz_mesh_mesh_edges_pub_;
  rclcpp::Publisher<MarkerMsg>::SharedPtr viz_pose_mesh_edges_pub_;

  // Transform broadcaster
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcast_;

  // Subscribers
  pose_graph_tools::PoseGraphSubscription pose_graph_incremental_sub_;
  rclcpp::Subscription<MeshMsg>::SharedPtr full_mesh_sub_;
  pose_graph_tools::PoseGraphSubscription incremental_mesh_graph_sub_;
  rclcpp::Subscription<PathMsg>::SharedPtr path_callback_sub_;

  // Service
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_mesh_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_traj_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr save_graph_srv_;
  rclcpp::Service<kimera_pgmo_msgs::srv::LoadGraphMesh>::SharedPtr load_graph_mesh_srv_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reset_srv_;

  // Trajectory
  Path trajectory_;
  std::vector<size_t> unconnected_mesh_indices_;
  std::vector<uint64_t> unconnected_mesh_index_stamps_;
  std::vector<Timestamp> timestamps_;
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
