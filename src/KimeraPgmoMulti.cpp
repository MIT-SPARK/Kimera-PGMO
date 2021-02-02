/**
 * @file   KimeraPgmoMulti.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include <chrono>
#include <cmath>
#include <map>
#include <vector>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include "kimera_pgmo/KimeraPgmoMulti.h"

namespace kimera_pgmo {

// Constructor
KimeraPgmoMulti::KimeraPgmoMulti() : KimeraPgmo() {}

KimeraPgmoMulti::~KimeraPgmoMulti() {}

/*! \brief Load the parameters required by this class through ROS
 *  - n: ROS node handle
 */
bool KimeraPgmoMulti::loadParameters(const ros::NodeHandle& n) {
  if (!KimeraPgmo::loadParameters(n)) return false;

  // Get number of robots
  if (!n.getParam("num_robots", num_robots_)) return false;
  return true;
}

/*! \brief Creates the ROS publishers used
 *  - n: ROS node handle
 */
bool KimeraPgmoMulti::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  // Optimized mesh publisher
  for (size_t i = 0; i < num_robots_; i++) {
    std::string topic_name = "kimera" + std::to_string(i) + "/optimized_mesh";
    optimized_mesh_pub_.push_back(
        nl.advertise<mesh_msgs::TriangleMeshStamped>(topic_name, 1, false));
  }
  pose_graph_pub_ =
      nl.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);
  viz_deformation_graph_pub_ =
      nl.advertise<visualization_msgs::Marker>("deformation_graph", 10, false);
  return true;
}

/*! \brief Starts the callbacks in this class
 *  - n: ROS node handle
 */
bool KimeraPgmoMulti::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  for (size_t i = 0; i < num_robots_; i++) {
    std::string full_mesh_topic =
        "kimera" + std::to_string(i) + "voxblox_processing/full_mesh";
    full_mesh_sub_.push_back(nl.subscribe(full_mesh_topic,
                                          1,
                                          &KimeraPgmoMulti::fullMeshCallback,
                                          dynamic_cast<KimeraPgmo*>(this)));

    std::string inc_mesh_topic =
        "kimera" + std::to_string(i) + "/voxblox_processing/partial_mesh";
    incremental_mesh_sub_.push_back(
        nl.subscribe(inc_mesh_topic,
                     5,
                     &KimeraPgmoMulti::incrementalMeshCallback,
                     dynamic_cast<KimeraPgmo*>(this)));

    std::string pose_graph_topic =
        "kimera" + std::to_string(i) + "kimera_vio_ros/pose_graph_incremental";
    pose_graph_incremental_sub_.push_back(
        nl.subscribe(pose_graph_topic,
                     1000,
                     &KimeraPgmoMulti::incrementalPoseGraphCallback,
                     dynamic_cast<KimeraPgmo*>(this)));

    std::string path_topic = "kimera" + std::to_string(i) + "/path";
    path_callback_sub_.push_back(
        nl.subscribe(path_topic,
                     2,
                     &KimeraPgmoMulti::optimizedPathCallback,
                     dynamic_cast<KimeraPgmo*>(this)));
  }

  // Initialize save mesh service
  save_mesh_srv_ = nl.advertiseService("save_mesh",
                                       &KimeraPgmoMulti::saveMeshCallback,
                                       dynamic_cast<KimeraPgmo*>(this));

  // Initialize save trajectory service
  save_traj_srv_ = nl.advertiseService("save_trajectory",
                                       &KimeraPgmoMulti::saveTrajectoryCallback,
                                       dynamic_cast<KimeraPgmo*>(this));
  return true;
}
}  // namespace kimera_pgmo