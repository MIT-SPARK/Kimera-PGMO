/**
 * @file   MesherMapper.h
 * @brief  MesherMapper class: Main class and ROS interface
 * @author Yun Chang
 */

#include <string>

#include <nav_msgs/Path.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <gtsam/geometry/Pose3.h>

#include "mesher_mapper/AbsolutePoseStamped.h"
#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/DeformationGraph.h"
#include "mesher_mapper/OctreeCompression.h"

namespace mesher_mapper {
class MesherMapper {
 public:
  MesherMapper();
  ~MesherMapper();

  // Initialize parameters, publishers, and subscribers
  bool Initialize(const ros::NodeHandle& n);

 private:
  bool LoadParameters(const ros::NodeHandle& n);

  bool CreatePublishers(const ros::NodeHandle& n);

  bool RegisterCallbacks(const ros::NodeHandle& n);

  // Functions to publish
  bool PublishOptimizedMesh();

  // Callback for loopclosure
  void LoopClosureCallback(
      const mesher_mapper::AbsolutePoseStamped::ConstPtr& msg);

  // Callback for recieved trajectory
  void TrajectoryCallback(const nav_msgs::Path::ConstPtr& msg);

  // Callback for mesh input
  void MeshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);

  // Timer callback
  void ProcessTimerCallback(const ros::TimerEvent& ev);

  pcl::PolygonMesh optimized_mesh_;

  // To compress the map (environment) mesh
  OctreeCompression map_compression_;
  // To get the simplified mesh for deformation graph
  OctreeCompression d_graph_compression_;

  DeformationGraph deformation_graph_;

  // Publishers
  ros::Publisher optimized_mesh_pub_;

  // Subscribers
  ros::Subscriber deform_input_sub_;

  // Trajectory
  std::vector<gtsam::Pose3> trajectory_;

  // Timer
  ros::Timer update_timer_;

  std::string frame_id_;
};
}  // namespace mesher_mapper
