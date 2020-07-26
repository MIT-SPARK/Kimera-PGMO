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
#include <std_srvs/Empty.h>

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

  // Save mesh service callback
  bool SaveMeshCallback(std_srvs::Empty::Request&, std_srvs::Empty::Response&);

  // Save optimized trajectory callback
  bool SaveTrajectoryCallback(std_srvs::Empty::Request&,
                              std_srvs::Empty::Response&);

  pcl::PolygonMesh input_mesh_;
  pcl::PolygonMesh optimized_mesh_;
  ros::Time last_mesh_stamp_;

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

  // Service
  ros::ServiceServer save_mesh_srv_;
  ros::ServiceServer save_traj_srv_;

  // Trajectory
  std::vector<gtsam::Pose3> trajectory_;
  std::vector<ros::Time> timestamps_;

  std::string frame_id_;

  // Parameters for embedding
  double embed_delta_t_;
  double embed_delta_r_;
  double timer_period_;

  // Save output
  std::string output_prefix_;
};
}  // namespace kimera_pgmo
