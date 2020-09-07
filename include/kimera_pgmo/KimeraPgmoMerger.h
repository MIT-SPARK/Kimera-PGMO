/**
 * @file   KimeraPgmoMerger.h
 * @brief  KimeraPgmoMerger class: Main class and ROS interface
 * @author Yun Chang
 */
#pragma once

#include <map>
#include <queue>
#include <string>

#include <nav_msgs/Odometry.h>
#include <pcl/PolygonMesh.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "kimera_pgmo/AbsolutePoseStamped.h"
#include "kimera_pgmo/DeformationGraph.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {
class KimeraPgmoMerger {
  friend class KimeraPgmoMergerTest;

 public:
  /*! \brief Constructor for Kimera Pgmo class. Which subscribes to the
   * incremental mesh and pose graph to create the deformation graph and also
   * the full mesh to perform distortions and publish the optimzed distored mesh
   * and trajectory
   */
  KimeraPgmoMerger();
  ~KimeraPgmoMerger();

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

  /*! \brief Recieves latest pose graph edges and deformation graph edges
   *  - msg: new Pose Graph message consisting of the newest pose graph edges
   */
  void poseGraphEdgesCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg);

  /*! \brief Subscribes to the full mesh and deform it based on the deformation
   * graph. Then publish the deformed mesh, and also the optimized pose graph
   *  - mesh_msg: the full unoptimized mesh in mesh_msgs TriangleMeshStamped
   * format
   */
  void meshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);

  /*! \brief Saves mesh as a ply file. Triggers through a rosservice call and
   * saves to file [output_prefix_].ply
   */
  bool saveMeshCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  /*! \brief Saves all the trajectories of all robots to csv files. Triggers
   * through a rosservice call and saves to file [output_prefix_][robot_id].csv
   */
  bool saveTrajectoryCallback(std_srvs::Empty::Request&,
                              std_srvs::Empty::Response&);

  pcl::PolygonMesh optimized_mesh_;
  ros::Time last_mesh_stamp_;

  // Deformation graph to collect deformation graph edges and pose graph edges
  DeformationGraph deformation_graph_;
  // Compress output merged mesh
  OctreeCompressionPtr compression_;

  // Publishers
  ros::Publisher optimized_mesh_pub_;
  ros::Publisher pose_graph_pub_;

  // Subscribers
  std::vector<ros::Subscriber> pose_graph_edges_sub_;
  std::vector<ros::Subscriber> mesh_sub_;
  ros::Subscriber incremental_mesh_sub_;

  // Service
  ros::ServiceServer save_mesh_srv_;
  ros::ServiceServer save_traj_srv_;

  // Trajectory
  std::vector<gtsam::Pose3> trajectory_;
  std::queue<gtsam::Key> unconnected_nodes_;
  std::vector<ros::Time> timestamps_;
  double embed_delta_t_;
  // maximum time allowed when associating node to mesh

  std::string frame_id_;
  size_t robot_id_;  // robot id for kimera pgmo instance

  double timer_period_;

  // Save output
  std::string output_prefix_;
};
}  // namespace kimera_pgmo
