/**
 * @file   KimeraPgmo.h
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */

#include <string>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <gtsam/geometry/Pose3.h>

#include "kimera_pgmo/AbsolutePoseStamped.h"
#include "kimera_pgmo/DeformationGraph.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {
class KimeraPgmo {
 public:
  KimeraPgmo();
  ~KimeraPgmo();

  // Initialize parameters, publishers, and subscribers
  bool Initialize(const ros::NodeHandle& n);

 private:
  bool LoadParameters(const ros::NodeHandle& n);

  bool CreatePublishers(const ros::NodeHandle& n);

  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Functions to publish
  bool PublishOptimizedMesh();
  bool PublishOptimizedPath() const;

  // Callback for loopclosure
  void IncrementalPoseGraphCallback(
      const pose_graph_tools::PoseGraph::ConstPtr& msg);

  // Callback for mesh input
  void MeshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);

  // Timer callback
  void ProcessTimerCallback(const ros::TimerEvent& ev);

  pcl::PolygonMesh input_mesh_;
  pcl::PolygonMesh optimized_mesh_;

  // To get the simplified mesh for deformation graph
  OctreeCompression d_graph_compression_;

  DeformationGraph deformation_graph_;

  // Publishers
  ros::Publisher optimized_mesh_pub_;
  ros::Publisher optimized_path_pub_;
  ros::Publisher optimized_odom_pub_;
  ros::Publisher pose_graph_pub_;

  // Subscribers
  ros::Subscriber pose_graph_incremental_sub_;
  ros::Subscriber input_mesh_sub_;

  // Trajectory
  std::vector<gtsam::Pose3> trajectory_;
  std::vector<ros::Time> timestamps_;

  // Timer
  ros::Timer update_timer_;

  std::string frame_id_;

  // Parameters for embedding
  double embed_delta_t_;
  double embed_delta_r_;
  double timer_rate_;

  // Save output
  bool save_optimized_mesh_;
  std::string output_file_;
};
}  // namespace kimera_pgmo