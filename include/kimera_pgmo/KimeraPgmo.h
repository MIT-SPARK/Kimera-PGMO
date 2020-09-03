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

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "kimera_pgmo/AbsolutePoseStamped.h"
#include "kimera_pgmo/DeformationGraph.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {
class KimeraPgmo {
  friend class KimeraPgmoTest;

 public:
  KimeraPgmo();
  ~KimeraPgmo();

  // Initialize parameters, publishers, and subscribers
  bool initialize(const ros::NodeHandle& n);

 protected:
  bool loadParameters(const ros::NodeHandle& n);

  bool createPublishers(const ros::NodeHandle& n);

  bool registerCallbacks(const ros::NodeHandle& n);

  // Functions to publish
  bool publishOptimizedMesh();
  bool publishOptimizedPath(const size_t& robot_id) const;

  // Callback for loopclosure
  void incrementalPoseGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& msg);

  // Callback for mesh input
  void fullMeshCallback(
      const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);

  void incrementalMeshCallback(
      const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);

  // Save mesh service callback
  bool saveMeshCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  // Save optimized trajectory callback
  bool saveTrajectoryCallback(std_srvs::Empty::Request&,
                              std_srvs::Empty::Response&);

  pcl::PolygonMesh input_mesh_;
  pcl::PolygonMesh optimized_mesh_;
  ros::Time last_mesh_stamp_;

  // To get the simplified mesh for deformation graph
  // OctreeCompression d_graph_compression_;

  DeformationGraph deformation_graph_;
  OctreeCompressionPtr compression_;
  double compression_time_horizon_;

  // Publishers
  ros::Publisher optimized_mesh_pub_;
  ros::Publisher optimized_path_pub_;  // currently unused
  ros::Publisher optimized_odom_pub_;
  ros::Publisher pose_graph_pub_;

  // Subscribers
  ros::Subscriber pose_graph_incremental_sub_;
  ros::Subscriber full_mesh_sub_;
  ros::Subscriber incremental_mesh_sub_;

  // Service
  ros::ServiceServer save_mesh_srv_;
  ros::ServiceServer save_traj_srv_;

  // Trajectory
  std::map<size_t, std::vector<gtsam::Pose3> > trajectory_;
  std::queue<gtsam::Key> unconnected_nodes_;
  std::map<size_t, std::vector<ros::Time> > timestamps_;
  double embed_delta_t_;
  // maximum time allowed when associating node to mesh

  std::string frame_id_;

  double timer_period_;

  // Save output
  std::string output_prefix_;
};
}  // namespace kimera_pgmo
