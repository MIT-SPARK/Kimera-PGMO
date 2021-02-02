/**
 * @file   KimeraPgmoMulti.h
 * @brief  KimeraPgmoMulti class: Derived multirobot class
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

#include "kimera_pgmo/KimeraPgmo.h"

namespace kimera_pgmo {

class KimeraPgmoMulti : public KimeraPgmo {
  friend class KimeraPgmoMultiTest;

 public:
  /*! \brief Constructor for Kimera Pgmo class. Which subscribes to the
   * incremental mesh and pose graph to create the deformation graph and also
   * the full mesh to perform distortions and publish the optimzed distored mesh
   * and trajectory
   */
  KimeraPgmoMulti();
  ~KimeraPgmoMulti();

 protected:
  /*! \brief Load the parameters required by this class through ROS
   *  - n: ROS node handle
   */
  bool loadParameters(const ros::NodeHandle& n) override;

  /*! \brief Creates the ROS publishers used
   *  - n: ROS node handle
   */
  bool createPublishers(const ros::NodeHandle& n) override;

  /*! \brief Starts the callbacks in this class
   *  - n: ROS node handle
   */
  bool registerCallbacks(const ros::NodeHandle& n) override;

 protected:
  int num_robots_;

  // Publishers
  std::vector<ros::Publisher> optimized_mesh_pub_;
  ros::Publisher pose_graph_pub_;

  // Subscribers
  std::vector<ros::Subscriber> pose_graph_incremental_sub_;
  std::vector<ros::Subscriber> full_mesh_sub_;
  std::vector<ros::Subscriber> incremental_mesh_sub_;
  std::vector<ros::Subscriber> path_callback_sub_;
};
}  // namespace kimera_pgmo
