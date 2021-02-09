/**
 * @file   KimeraPgmoMulti.cpp
 * @brief  KimeraPgmoMulti class: Main class and ROS interface
 * @author Yun Chang
 */
#include <chrono>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include "kimera_pgmo/KimeraPgmoMulti.h"

namespace kimera_pgmo {

// Constructor
KimeraPgmoMulti::KimeraPgmoMulti(const std::vector<size_t>& robot_ids)
    : num_loop_closures_(0),
      inc_mesh_cb_time_(0),
      full_mesh_cb_time_(0),
      pg_cb_time_(0),
      path_cb_time_(0),
      robot_ids_(robot_ids) {}

KimeraPgmoMulti::~KimeraPgmoMulti() {}

// Initialize parameters, publishers, and subscribers and deformation graph
bool KimeraPgmoMulti::initialize(const ros::NodeHandle& n) {
  if (!loadParameters(n)) {
    ROS_ERROR("KimeraPgmoMulti: Failed to load parameters.");
  }

  if (!createPublishers(n)) {
    ROS_ERROR("KimeraPgmoMulti: Failed to create publishers.");
  }

  if (!registerCallbacks(n)) {
    ROS_ERROR("KimeraPgmoMulti: Failed to register callbacks.");
  }

  // Log header to file
  if (log_output_) {
    std::string log_file = output_prefix_ + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  ROS_INFO("Initialized Kimera-PGMO-Multi.");

  return true;
}

// Load deformation parameters
bool KimeraPgmoMulti::loadParameters(const ros::NodeHandle& n) {
  if (!KimeraPgmoInterface::loadParameters(n)) return false;

  if (!n.getParam("frame_id", frame_id_)) return false;
  if (n.getParam("output_prefix", output_prefix_)) {
    ROS_INFO("Saving optimized data to: %s/ mesh_pgmo.ply and traj_pgmo.csv",
             output_prefix_.c_str());
    n.getParam("log_output", log_output_);
    if (log_output_) {
      ROS_INFO("Logging output to: %s/kimera_pgmo_log.csv",
               output_prefix_.c_str());
    }
  }

  // start the mesh compression module for deformation graph
  if (!n.getParam("d_graph_resolution", deformation_graph_resolution_))
    return false;

  return true;
}

// Initialize publishers
bool KimeraPgmoMulti::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  pose_graph_pub_ =
      nl.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);
  viz_deformation_graph_pub_ =
      nl.advertise<visualization_msgs::Marker>("deformation_graph", 10, false);
  // Optimized mesh and path publisher
  for (auto id : robot_ids_) {
    std::string mesh_topic_name =
        "kimera" + std::to_string(id) + "/optimized_mesh";
    optimized_mesh_pub_.push_back(nl.advertise<mesh_msgs::TriangleMeshStamped>(
        mesh_topic_name, 1, false));
    std::string path_topic_name =
        "kimera" + std::to_string(id) + "/optimized_path";
    optimized_path_pub_.push_back(
        nl.advertise<nav_msgs::Path>(path_topic_name, 1, false));
  }
  return true;
}

// Initialize callbacks
bool KimeraPgmoMulti::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  for (auto id : robot_ids_) {
    std::string full_mesh_topic =
        "/kimera" + std::to_string(id) + "/voxblox_processing/full_mesh";
    full_mesh_sub_.push_back(nl.subscribe(
        full_mesh_topic, 1, &KimeraPgmoMulti::fullMeshCallback, this));

    std::string inc_mesh_topic =
        "/kimera" + std::to_string(id) + "/voxblox_processing/partial_mesh";
    incremental_mesh_sub_.push_back(nl.subscribe(
        inc_mesh_topic, 5, &KimeraPgmoMulti::incrementalMeshCallback, this));

    std::string pose_graph_topic = "/kimera" + std::to_string(id) +
                                   "/kimera_vio_ros/pose_graph_incremental";
    pose_graph_incremental_sub_.push_back(
        nl.subscribe(pose_graph_topic,
                     1000,
                     &KimeraPgmoMulti::incrementalPoseGraphCallback,
                     this));

    // std::string path_topic = "/kimera" + std::to_string(i) + "/path";
    // TODO: Path callback
    // path_callback_sub_.push_back(nl.subscribe(
    //     path_topic, 2, &KimeraPgmoMulti::optimizedPathCallback, this));
  }

  // Initialize save mesh service
  save_mesh_srv_ = nl.advertiseService(
      "save_mesh", &KimeraPgmoMulti::saveMeshCallback, this);

  // Initialize save trajectory service
  save_traj_srv_ = nl.advertiseService(
      "save_trajectory", &KimeraPgmoMulti::saveTrajectoryCallback, this);
  return true;
}

// To publish optimized mesh
bool KimeraPgmoMulti::publishOptimizedMesh(const size_t& robot_id) const {
  std_msgs::Header msg_header;
  msg_header.stamp = last_mesh_stamp_.at(robot_id);
  msg_header.frame_id = frame_id_;
  publishMesh(
      optimized_mesh_.at(robot_id), msg_header, &optimized_mesh_pub_[robot_id]);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmoMulti::publishOptimizedPath(const size_t& robot_id) const {
  std::vector<gtsam::Pose3> gtsam_path =
      deformation_graph_.getOptimizedTrajectory(GetRobotPrefix(robot_id));

  if (gtsam_path.size() == 0) return false;

  std_msgs::Header msg_header;
  msg_header.stamp = ros::Time::now();
  msg_header.frame_id = frame_id_;
  return publishPath(gtsam_path, msg_header, &optimized_path_pub_[robot_id]);
}

void KimeraPgmoMulti::incrementalPoseGraphCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  if (msg->nodes.size() == 0 && msg->edges.size() == 0) return;
  size_t robot_id;
  if (msg->nodes.size() > 0) {
    robot_id = msg->nodes[0].robot_id;
  } else {
    robot_id = msg->edges[0].robot_from;
  }
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  // Check if robot is already in trajectory
  if (trajectory_.find(robot_id) == trajectory_.end()) {
    trajectory_[robot_id] = std::vector<gtsam::Pose3>{};
    unconnected_nodes_[robot_id] = std::queue<size_t>{};
    timestamps_[robot_id] = std::vector<ros::Time>{};
  }

  // std::map<size_t, std::vector<gtsam::Pose3> >::iterator trajectory_ptr =
  // trajectory_.find(robot_id); std::map<size_t, std::queue<size_t> >
  // unconnected_nodes_; std::map<size_t, std::vector<ros::Time> > timestamps_;
  processIncrementalPoseGraph(msg,
                              &trajectory_[robot_id],
                              &unconnected_nodes_[robot_id],
                              &timestamps_[robot_id]);

  // Update transforms
  publishTransforms();

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  pg_cb_time_ = spin_duration.count();

  // Log to file
  if (log_output_) {
    std::string log_file = output_prefix_ + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    // Publish pose graph
    const GraphMsgPtr& pose_graph_ptr =
        deformation_graph_.getPoseGraph(timestamps_);
    pose_graph_pub_.publish(*pose_graph_ptr);
  }
}

void KimeraPgmoMulti::optimizedPathCallback(
    const nav_msgs::Path::ConstPtr& path_msg) {
  // TODO: for this to work need something like nav_msgs::PathWithId
  // Start timer
  // auto start = std::chrono::high_resolution_clock::now();

  // if (trajectory_.size() > 1) {
  //   ROS_ERROR(
  //       "KimeraPgmoMulti: Path subscriber does not support centralized "
  //       "multirobot "
  //       "scenario. ");
  // }
  // processOptimizedPath(path_msg, trajectory_.begin()->first);

  // // Stop timer and save
  // auto stop = std::chrono::high_resolution_clock::now();
  // auto spin_duration =
  //     std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // path_cb_time_ = spin_duration.count();

  // // Log to file
  // if (log_output_) {
  //   std::string log_file = output_prefix_ +
  //   std::string("/kimera_pgmo_log.csv"); logStats(log_file);
  // }
}

void KimeraPgmoMulti::fullMeshCallback(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  optimized_mesh_[mesh_msg->id] =
      optimizeAndPublishFullMesh(mesh_msg, &optimized_mesh_pub_[mesh_msg->id]);

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  full_mesh_cb_time_ = spin_duration.count();

  return;
}

void KimeraPgmoMulti::incrementalMeshCallback(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  const size_t& robot_id = mesh_msg->id;

  // Check if the compressor is already initialized
  if (compression_.find(robot_id) == compression_.end()) {
    compression_[robot_id] = OctreeCompressionPtr(
        new OctreeCompression(deformation_graph_resolution_));
  }
  processIncrementalMesh(mesh_msg,
                         compression_[robot_id],
                         timestamps_[robot_id],
                         &unconnected_nodes_[robot_id]);

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  inc_mesh_cb_time_ = spin_duration.count();

  // Publish deformation graph visualization
  visualizeDeformationGraph(&viz_deformation_graph_pub_);

  return;
}

void KimeraPgmoMulti::publishTransforms() {
  for (auto traj : trajectory_) {
    const size_t& robot_id = traj.first;
    const std::vector<gtsam::Pose3>& gtsam_path =
        deformation_graph_.getOptimizedTrajectory(GetRobotPrefix(robot_id));
    const gtsam::Pose3& latest_pose = gtsam_path.at(traj.second.size() - 1);

    const gtsam::Point3& pos = latest_pose.translation();
    const gtsam::Quaternion& quat = latest_pose.rotation().toQuaternion();
    // Create transfomr message

    geometry_msgs::TransformStamped transform;
    std::string frame_name = "pgmo_base_link_" + std::to_string(robot_id);
    transform.header.stamp = ros::Time::now();
    transform.header.frame_id = "world";
    transform.child_frame_id = frame_name;
    transform.transform.translation.x = pos.x();
    transform.transform.translation.y = pos.y();
    transform.transform.translation.z = pos.z();
    transform.transform.rotation.x = quat.x();
    transform.transform.rotation.y = quat.y();
    transform.transform.rotation.z = quat.z();
    transform.transform.rotation.w = quat.w();

    tf_broadcast_.sendTransform(transform);
  }
}

bool KimeraPgmoMulti::saveMeshCallback(std_srvs::Empty::Request&,
                                       std_srvs::Empty::Response&) {
  // Save mesh
  for (auto mesh : optimized_mesh_) {
    std::string ply_name = output_prefix_ + std::string("/mesh_pgmo_") +
                           std::to_string(mesh.first) + std::string(".ply");
    saveMesh(mesh.second, ply_name);
  }
  ROS_INFO("KimeraPgmoMulti: Saved mesh to file.");
  return true;
}

bool KimeraPgmoMulti::saveTrajectoryCallback(std_srvs::Empty::Request&,
                                             std_srvs::Empty::Response&) {
  // Save trajectory
  for (auto traj : trajectory_) {
    const size_t& robot_id = traj.first;
    const std::vector<gtsam::Pose3>& optimized_path =
        deformation_graph_.getOptimizedTrajectory(GetRobotPrefix(robot_id));
    std::ofstream csvfile;
    std::string csv_name = output_prefix_ + std::string("/traj_pgmo_") +
                           std::to_string(robot_id) + std::string(".csv");
    saveTrajectory(optimized_path, timestamps_[robot_id], csv_name);
  }
  ROS_INFO("KimeraPgmoMulti: Saved trajectories to file.");
  return true;
}

void KimeraPgmoMulti::logStats(const std::string filename) const {
  std::ofstream file;

  if (trajectory_.size() < 1) {
    file.open(filename);
    // file format
    file << "num-robots,num-keyframes,num-loop-closures,total-num-factors,num-"
            "vertices,num-vertices-simplified,inc-mesh-cb-time(mu-s),full-mesh-"
            "cb-time(mu-s),pg-cb-time(mu-s),path-cb-time(mu-s)\n";
    return;
  }
  // Number of keyframes
  size_t num_keyframes = 0;
  for (auto traj : trajectory_) {
    num_keyframes = num_keyframes + traj.second.size();
  }

  // Number of vertices (total)
  size_t num_vertices = 0;
  for (auto mesh : optimized_mesh_) {
    num_vertices =
        num_vertices + mesh.second.cloud.width * mesh.second.cloud.height;
  }

  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << trajectory_.size() << "," << num_keyframes << ","
       << num_loop_closures_ << ","
       << deformation_graph_.getGtsamFactors().size() << "," << num_vertices
       << "," << deformation_graph_.getVertices().points.size() << ","
       << inc_mesh_cb_time_ << "," << full_mesh_cb_time_ << "," << pg_cb_time_
       << "," << path_cb_time_ << std::endl;
  file.close();
}

}  // namespace kimera_pgmo