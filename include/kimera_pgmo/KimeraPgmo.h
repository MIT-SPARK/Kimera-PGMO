/**
 * @file   KimeraPgmo.h
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#pragma once

#include "kimera_pgmo/AbsolutePoseStamped.h"
#include "kimera_pgmo/KimeraPgmoInterface.h"
#include "kimera_pgmo/KimeraPgmoMesh.h"
#include "kimera_pgmo/LoadGraphMesh.h"
#include "kimera_pgmo/RequestMeshFactors.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

#include <memory>
#include <mutex>
#include <thread>

namespace kimera_pgmo {
class KimeraPgmo : public KimeraPgmoInterface {
  friend class KimeraPgmoTest;
  friend class KimeraDpgmoTest;

 public:
  /*! \brief Constructor for Kimera Pgmo class. Which subscribes to the
   * incremental mesh and pose graph to create the deformation graph and also
   * the full mesh to perform distortions and publish the optimzed distored mesh
   * and trajectory
   */
  KimeraPgmo();
  ~KimeraPgmo();

  /*! \brief Initializes callbacks and publishers, and also parse the parameters
   *  - n: ROS node handle.
   */
  bool initialize(const ros::NodeHandle& n) override;

  /*! \brief Get a pointer to the optimized mesh
   */
  inline pcl::PolygonMesh::ConstPtr getOptimizedMeshPtr() const {
    return optimized_mesh_;
  }

  /*! \brief Get the current robot id
   */
  inline int getRobotId() const { return robot_id_; };

  /*! \brief Get the current robot prefix
   */
  inline char getRobotPrefix() const { return robot_id_to_prefix.at(robot_id_); }

  /*! \brief Get the timestamps for the robot
   */
  inline std::vector<ros::Time> getRobotTimestamps() const { return timestamps_; };

 protected:
  /*! \brief Start the thread doing the mesh graph / pose graph / path
   * subscription.
   *  - n: ROS node handle
   */
  void startGraphProcess(const ros::NodeHandle& n);

  /*! \brief Start the thread doing the full mesh subscription.
   *  - n: ROS node handle
   */
  void startMeshProcess(const ros::NodeHandle& n);

  /*! \brief Load the parameters required by this class through ROS
   *  - n: ROS node handle
   */
  bool loadParameters(const ros::NodeHandle& n) override;

  /*! \brief Creates the ROS publishers used
   *  - n: ROS node handle
   */
  bool createPublishers(const ros::NodeHandle& n) override;

  virtual bool registerCallbacks(const ros::NodeHandle&) override {
    return true;
  }

  /*! \brief Publish the optimized mesh (stored after deformation)
   */
  bool publishOptimizedMesh() const;

  /*! \brief Publish optimized trajectory (Currently unused, as trajectory can
   * be visualized with published pose graph)
   *  - robot_id: the robot for which the trajectory is to be published
   */
  bool publishOptimizedPath() const;

  /*! \brief Recieves latest edges in the pose graph and add to deformation
   * graph. Also place the received node in a queue to connect them to the
   * incremental mesh when that comes in
   *  - msg: new Pose Graph message consisting of the newest pose graph edges
   */
  void incrementalPoseGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& msg);

  /*! \brief Subscribes to the full mesh and deform it based on the deformation
   * graph. Then publish the deformed mesh, and also the optimized pose graph
   *  - mesh_msg: the full unoptimized mesh in mesh_msgs TriangleMeshStamped
   * format
   */
  void fullMeshCallback(const KimeraPgmoMesh::ConstPtr& mesh_msg);

  /*! \brief Publish the transform for each robot id based on the latest node in
   * pose graph
   */
  void publishTransforms();

  /*! \brief Subscribes to the mesh factors from MeshFrontend, which
   * corresponds to the latest simplified partial mesh from Voxblox or
   * Kimera-Semantics. We add to the deformation graph and also connect the
   * nodes stored in the waiting queue to the vertices of the sampled mesh,
   *  - mesh_graph_msg: mesh factors to add to deformation graph and mesh nodes.
   */
  void incrementalMeshGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& mesh_graph_msg);

  /*! \brief Subscribes to an optimized trajectory. The path should correspond
   * to the nodes of the pose graph received in the
   * incrementalPoseGraphCallback. Note that this should only be used in the
   * single robot pose graph case.
   *  - mesh_msg: partial mesh in mesh_msgs TriangleMeshStamped format
   */
  void optimizedPathCallback(const nav_msgs::Path::ConstPtr& path_msg);

  /*! \brief Subscribes to an optimized values published by dpgmo
   *  - msg: optimized pose graph published as a pose graph msg
   */
  void dpgmoCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  /*! \brief Saves mesh as a ply file. Triggers through a rosservice call
   * and saves to file [output_prefix_]/mesh_pgmo.ply
   */
  bool saveMeshCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /*! \brief Saves all the trajectories of all robots to csv files. Triggers
   * through a rosservice call and saves to file [output_prefix_]/traj_pgmo.csv
   */
  bool saveTrajectoryCallback(std_srvs::Empty::Request&,
                              std_srvs::Empty::Response&);

  /*! \brief Saves the deformation graph to a custom dgrf file. Triggers
   * through a rosservice call and saves to file [output_prefix_]/pgmo.dgrf
   */
  bool saveGraphCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /*! \brief Loads a deformation graph and associated mesh.
   */
  bool loadGraphMeshCallback(kimera_pgmo::LoadGraphMesh::Request& request,
                             kimera_pgmo::LoadGraphMesh::Response& response);

  /*! \brief Requests the mesh related edges (pose-vertex, vertex-vertex) in the
   * deformation graph.
   */
  bool requestMeshEdgesCallback(
      kimera_pgmo::RequestMeshFactors::Request& request,
      kimera_pgmo::RequestMeshFactors::Response& response);

  /*! \brief log the run-time stats such as pose graph size, mesh size, and run
   * time
   */
  void logStats(const std::string filename) const;

  /*! \brief Clear and reset the deformation graph.
   */
  bool resetGraphCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
    resetDeformationGraph();
    return true;
  }

 protected:
  // optimized mesh for each robot
  pcl::PolygonMesh::Ptr optimized_mesh_;
  std::vector<ros::Time> mesh_vertex_stamps_;

  PathPtr optimized_path_;
  ros::Time last_mesh_stamp_;

  // Publishers
  ros::Publisher optimized_mesh_pub_;
  ros::Publisher optimized_path_pub_;  // Unused for now (TODO)
  ros::Publisher optimized_odom_pub_;  // Unused for now (TODO)
  ros::Publisher pose_graph_pub_;
  ros::Publisher viz_mesh_mesh_edges_pub_;
  ros::Publisher viz_pose_mesh_edges_pub_;

  // Transform broadcaster
  tf2_ros::TransformBroadcaster tf_broadcast_;

  // Subscribers
  ros::Subscriber pose_graph_incremental_sub_;
  ros::Subscriber full_mesh_sub_;
  ros::Subscriber incremental_mesh_graph_sub_;
  ros::Subscriber path_callback_sub_;
  ros::Subscriber dpgmo_callback_sub_;

  // Service
  ros::ServiceServer save_mesh_srv_;
  ros::ServiceServer save_traj_srv_;
  ros::ServiceServer save_graph_srv_;
  ros::ServiceServer load_graph_mesh_srv_;
  ros::ServiceServer reset_srv_;
  ros::ServiceServer req_mesh_edges_srv_;

  // Trajectory
  Path trajectory_;
  std::queue<size_t> unconnected_nodes_;
  std::vector<ros::Time> timestamps_;
  std::queue<size_t> dpgmo_num_poses_last_req_;

  std::string frame_id_;
  int robot_id_;

  std::unique_ptr<std::thread> graph_thread_;
  std::unique_ptr<std::thread> mesh_thread_;
  std::mutex interface_mutex_;

  // Time callback spin time
  int inc_mesh_cb_time_;
  int full_mesh_cb_time_;
  int pg_cb_time_;
  int path_cb_time_;

  // Save output
  std::string output_prefix_;
  // Log output to output_prefix_ folder
  bool log_output_;
};
}  // namespace kimera_pgmo
