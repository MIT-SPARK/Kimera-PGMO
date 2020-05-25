/**
 * @file   MesherMapper.h
 * @brief  MesherMapper class: Main class and ROS interface
 * @author Yun Chang
 */

#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <gtsam/nonlinear/NonlinearFactorGraph.h>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/DeformationGraph.h"
#include "mesher_mapper/OctreeCompression.h"
#include "mesher_mapper/RelativePoseStamped.h"

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
  bool PublishOptimizedMesh(const pcl::PolygonMesh& mesh);

  // Callback for loopclosure
  bool LoopClosureCallback(
      const mesher_mapper::RelativePoseStamped::ConstPtr& msg);

  // Callback for mesh input
  void MeshCallback(const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg);

  pcl::PolygonMesh input_mesh_;
  pcl::PolygonMesh optimized_mesh_;

  OctreeCompression compression_;
  DeformationGraph deformation_graph_;

  // Publishers
  ros::Publisher optimized_mesh_pub_;

  // Subscribers
  ros::Subscriber input_mesh_sub_;
  ros::Subscriber trajectory_sub_;
};
}  // namespace mesher_mapper
