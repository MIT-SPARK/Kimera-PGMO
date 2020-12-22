/**
 * @file   KimeraPgmo.h
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#pragma once

#include <map>
#include <queue>
#include <string>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "kimera_pgmo/AbsolutePoseStamped.h"
#include "kimera_pgmo/DeformationGraph.h"
#include "kimera_pgmo/TriangleMeshIdStamped.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {
class KimeraPgmo {
  friend class KimeraPgmoTest;

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
  bool initialize(const ros::NodeHandle& n);

 protected:
  /*! \brief Load the parameters required by this class through ROS
   *  - n: ROS node handle
   */
  bool loadParameters(const ros::NodeHandle& n);

  /*! \brief Creates the ROS publishers used
   *  - n: ROS node handle
   */
  bool createPublishers(const ros::NodeHandle& n);

  /*! \brief Starts the callbacks in this class
   *  - n: ROS node handle
   */
  bool registerCallbacks(const ros::NodeHandle& n);

  /*! \brief Publish the optimized mesh (stored after deformation)
   */
  bool publishOptimizedMesh();

  /*! \brief Publish optimized trajectory (Currently unused, as trajectory can
   * be visualized with published pose graph)
   *  - robot_id: the robot for which the trajectory is to be published
   */
  bool publishOptimizedPath(const size_t& robot_id) const;

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
  void fullMeshCallback(
      const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg);

  /*! \brief Publish the transform for each robot id based on the latest node in
   * pose graph
   */
  void publishTransforms();

  /*! \brief Subscribes to the partial mesh from VoxbloxProcessing, which
   * corresponds to the latest partial mesh from Voxblox or Kimera-Semantics. We
   * sample this partial mesh to add to the deformation graph and also connect
   * the nodes stored in the waiting queue to the vertices of the sampled mesh,
   * provided that the time difference is within the threshold
   *  - mesh_msg: partial mesh in mesh_msgs TriangleMeshStamped format
   */
  void incrementalMeshCallback(
      const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg);

  /*! \brief Subscribes to an optimized trajectory. The path should correspond
   * to the nodes of the pose graph received in the
   * incrementalPoseGraphCallback. Note that this should only be used in the
   * single robot pose graph case.
   *  - mesh_msg: partial mesh in mesh_msgs TriangleMeshStamped format
   */
  void optimizedPathCallback(const nav_msgs::Path::ConstPtr& path_msg);

  /*! \brief Saves mesh as a ply file. Triggers through a rosservice call
   * and saves to file [output_prefix_].ply
   */
  bool saveMeshCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /*! \brief Saves all the trajectories of all robots to csv files. Triggers
   * through a rosservice call and saves to file [output_prefix_][robot_id].csv
   */
  bool saveTrajectoryCallback(std_srvs::Empty::Request&,
                              std_srvs::Empty::Response&);

  /*! \brief log the run-time stats such as pose graph size, mesh size, and run
   * time
   */
  void logStats(const std::string filename) const;

 protected:
  enum class RunMode {
    FULL = 0u,  // Optimize mesh and pose graph
    MESH = 1u   // Optimize mesh based on given optimized trajectory
  };
  RunMode run_mode_;
  bool use_msg_time_;  // use msg time or call back time

  pcl::PolygonMesh input_mesh_;
  pcl::PolygonMesh optimized_mesh_;
  ros::Time last_mesh_stamp_;

  DeformationGraph deformation_graph_;
  OctreeCompressionPtr compression_;
  double compression_time_horizon_;

  // Publishers
  ros::Publisher optimized_mesh_pub_;
  ros::Publisher optimized_path_pub_;  // Unused for now (TODO)
  ros::Publisher optimized_odom_pub_;  // Unused for now (TODO)
  ros::Publisher pose_graph_pub_;

  // Transform broadcaster
  tf2_ros::TransformBroadcaster tf_broadcast_;

  // Subscribers
  ros::Subscriber pose_graph_incremental_sub_;
  ros::Subscriber full_mesh_sub_;
  ros::Subscriber incremental_mesh_sub_;
  ros::Subscriber path_callback_sub_;

  // Service
  ros::ServiceServer save_mesh_srv_;
  ros::ServiceServer save_traj_srv_;

  // Trajectory
  std::map<size_t, std::vector<gtsam::Pose3> > trajectory_;
  std::map<size_t, std::queue<size_t> > unconnected_nodes_;
  std::map<size_t, std::vector<ros::Time> > timestamps_;
  double embed_delta_t_;
  // maximum time allowed when associating node to mesh

  std::string frame_id_;

  double timer_period_;

  // Track number of loop closures
  size_t num_loop_closures_;

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
