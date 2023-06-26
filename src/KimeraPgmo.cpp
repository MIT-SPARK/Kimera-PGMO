/**
 * @file   KimeraPgmo.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include <chrono>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include "kimera_pgmo/KimeraPgmo.h"

namespace kimera_pgmo {

// Constructor
KimeraPgmo::KimeraPgmo()
    : inc_mesh_cb_time_(0),
      full_mesh_cb_time_(0),
      pg_cb_time_(0),
      path_cb_time_(0),
      optimized_path_(new Path),
      optimized_mesh_(new pcl::PolygonMesh) {}

KimeraPgmo::~KimeraPgmo() {
  if (graph_thread_) {
    graph_thread_->join();
    graph_thread_.reset();
  }

  if (mesh_thread_) {
    mesh_thread_->join();
    mesh_thread_.reset();
  }
}

// Initialize parameters, publishers, and subscribers and deformation graph
bool KimeraPgmo::initialize(const ros::NodeHandle& n) {
  if (!loadParameters(n)) {
    ROS_ERROR("KimeraPgmo: Failed to load parameters.");
  }

  if (!createPublishers(n)) {
    ROS_ERROR("KimeraPgmo: Failed to create publishers.");
  }

  // Log header to file
  if (log_output_) {
    std::string log_file = config_.log_path + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  // Start graph thread
  graph_thread_.reset(new std::thread(&KimeraPgmo::startGraphProcess, this, n));

  // Start full mesh thread
  mesh_thread_.reset(new std::thread(&KimeraPgmo::startMeshProcess, this, n));

  ROS_INFO("Initialized Kimera-PGMO.");

  return true;
}

// Load deformation parameters
bool KimeraPgmo::loadParameters(const ros::NodeHandle& n) {
  if (!KimeraPgmoInterface::loadParameters(n)) return false;

  if (!n.getParam("frame_id", frame_id_)) return false;
  if (!n.getParam("robot_id", robot_id_)) return false;

  if (config_.log_path != "") {
    ROS_INFO_STREAM("Saving optimized data to: "
                    << config_.log_path << "/ mesh_pgmo.ply and traj_pgmo.csv");
    n.getParam("log_output", log_output_);
    if (log_output_) {
      ROS_INFO_STREAM("Logging output to: " << config_.log_path
                                            << "/kimera_pgmo_log.csv");
    }
  }
  return true;
}

// Initialize publishers
bool KimeraPgmo::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  optimized_mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("optimized_mesh", 1, false);
  optimized_odom_pub_ = nl.advertise<nav_msgs::Odometry>("optimized_odom", 1, false);
  pose_graph_pub_ = nl.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);
  optimized_path_pub_ = nl.advertise<nav_msgs::Path>("optimized_path", 1, false);
  viz_mesh_mesh_edges_pub_ = nl.advertise<visualization_msgs::Marker>(
      "deformation_graph_mesh_mesh", 10, false);
  viz_pose_mesh_edges_pub_ = nl.advertise<visualization_msgs::Marker>(
      "deformation_graph_pose_mesh", 10, false);
  return true;
}

// Initialize callbacks
void KimeraPgmo::startGraphProcess(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  incremental_mesh_graph_sub_ = nl.subscribe(
      "mesh_graph_incremental", 5000, &KimeraPgmo::incrementalMeshGraphCallback, this);

  pose_graph_incremental_sub_ = nl.subscribe(
      "pose_graph_incremental", 5000, &KimeraPgmo::incrementalPoseGraphCallback, this);

  path_callback_sub_ =
      nl.subscribe("input_path", 2, &KimeraPgmo::optimizedPathCallback, this);

  dpgmo_callback_sub_ =
      nl.subscribe("optimized_values", 1, &KimeraPgmo::dpgmoCallback, this);

  // Initialize save trajectory service
  save_traj_srv_ =
      nl.advertiseService("save_trajectory", &KimeraPgmo::saveTrajectoryCallback, this);

  // Initialize save deformation graph service
  save_graph_srv_ =
      nl.advertiseService("save_dgrf", &KimeraPgmo::saveGraphCallback, this);

  // Initialize save deformation graph service
  load_graph_mesh_srv_ =
      nl.advertiseService("load_graph_mesh", &KimeraPgmo::loadGraphMeshCallback, this);

  // Reset the deformation graph service
  reset_srv_ =
      nl.advertiseService("reset_graph", &KimeraPgmo::resetGraphCallback, this);

  // Initialize request mesh edges service
  req_mesh_edges_srv_ = nl.advertiseService(
      "get_mesh_edges", &KimeraPgmo::requestMeshEdgesCallback, this);
}

// Initialize callbacks
void KimeraPgmo::startMeshProcess(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  full_mesh_sub_ = nl.subscribe("full_mesh", 1, &KimeraPgmo::fullMeshCallback, this);

  // Initialize save mesh service
  save_mesh_srv_ =
      nl.advertiseService("save_mesh", &KimeraPgmo::saveMeshCallback, this);
}

// To publish optimized mesh
bool KimeraPgmo::publishOptimizedMesh() const {
  std_msgs::Header msg_header;
  msg_header.stamp = last_mesh_stamp_;
  msg_header.frame_id = frame_id_;
  publishMesh(*optimized_mesh_, msg_header, &optimized_mesh_pub_);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmo::publishOptimizedPath() const {
  if (optimized_path_->size() == 0) return false;

  std_msgs::Header msg_header;
  msg_header.stamp = timestamps_.back();
  msg_header.frame_id = frame_id_;
  publishPath(*optimized_path_, msg_header, &optimized_path_pub_);

  if (optimized_odom_pub_.getNumSubscribers() > 0) {
    // Publish also the optimized odometry
    nav_msgs::Odometry odometry_msg;
    const gtsam::Pose3 last_pose = optimized_path_->at(optimized_path_->size() - 1);
    const gtsam::Rot3& rotation = last_pose.rotation();
    const gtsam::Quaternion& quaternion = rotation.toQuaternion();

    // Create header.
    odometry_msg.header.stamp = timestamps_.back();
    odometry_msg.header.frame_id = frame_id_;

    // Position
    odometry_msg.pose.pose.position.x = last_pose.x();
    odometry_msg.pose.pose.position.y = last_pose.y();
    odometry_msg.pose.pose.position.z = last_pose.z();

    // Orientation
    odometry_msg.pose.pose.orientation.w = quaternion.w();
    odometry_msg.pose.pose.orientation.x = quaternion.x();
    odometry_msg.pose.pose.orientation.y = quaternion.y();
    odometry_msg.pose.pose.orientation.z = quaternion.z();

    optimized_odom_pub_.publish(odometry_msg);
  }
  return true;
}

void KimeraPgmo::incrementalPoseGraphCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  if (msg->nodes.size() == 0 && msg->edges.size() == 0) return;
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    processIncrementalPoseGraph(msg, &trajectory_, &unconnected_nodes_, &timestamps_);
    // Update optimized path
    *optimized_path_ = getOptimizedTrajectory(robot_id_);
  }  // end interface critical section
  // Update transforms
  publishTransforms();

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  pg_cb_time_ = spin_duration.count();

  // Log to file
  if (log_output_) {
    std::string log_file = config_.log_path + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    // Publish pose graph
    std::map<size_t, std::vector<ros::Time> > id_timestamps;
    id_timestamps[robot_id_] = timestamps_;
    const GraphMsgPtr& pose_graph_ptr = deformation_graph_->getPoseGraph(id_timestamps);
    pose_graph_pub_.publish(*pose_graph_ptr);
  }

  // Publish optimized trajectory
  publishOptimizedPath();
}

void KimeraPgmo::optimizedPathCallback(const nav_msgs::Path::ConstPtr& path_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    processOptimizedPath(path_msg, robot_id_);
  }  // end interface critical section

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  path_cb_time_ = spin_duration.count();

  // Log to file
  if (log_output_) {
    std::string log_file = config_.log_path + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }
}

void KimeraPgmo::fullMeshCallback(
    const kimera_pgmo::KimeraPgmoMesh::ConstPtr& mesh_msg) {
  auto start = std::chrono::high_resolution_clock::now();
  bool opt_mesh;
  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    // Optimization always happen here only to ensure that the full mesh is
    // always optimized when published
    opt_mesh = optimizeFullMesh(*mesh_msg, optimized_mesh_, &mesh_vertex_stamps_, true);
  }  // end interface critical section
  if (opt_mesh && optimized_mesh_pub_.getNumSubscribers() > 0) {
    std_msgs::Header msg_header = mesh_msg->header;
    publishMesh(*optimized_mesh_, msg_header, &optimized_mesh_pub_);
  }
  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  full_mesh_cb_time_ = spin_duration.count();

  // Publish deformation graph edges visualization
  visualizeDeformationGraphMeshEdges(&viz_mesh_mesh_edges_pub_,
                                     &viz_pose_mesh_edges_pub_);
  return;
}

void KimeraPgmo::incrementalMeshGraphCallback(
    const pose_graph_tools::PoseGraph::ConstPtr& mesh_graph_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    processIncrementalMeshGraph(mesh_graph_msg, timestamps_, &unconnected_nodes_);
  }  // end interface critical section
  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  inc_mesh_cb_time_ = spin_duration.count();

  return;
}

void KimeraPgmo::dpgmoCallback(const pose_graph_tools::PoseGraph::ConstPtr& msg) {
  if (dpgmo_num_poses_last_req_.empty()) {
    ROS_ERROR("Mesh factors request queue empty.");
    return;
  }
  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    size_t num_poses = dpgmo_num_poses_last_req_.front();
    dpgmo_num_poses_last_req_.pop();
    for (auto node : msg->nodes) {
      if (node.robot_id != robot_id_) {
        ROS_WARN("Unexpected robot id in pose graph received in dpgmo callback. ");
        continue;
      }
      char prefix = robot_id_to_prefix.at(robot_id_);
      size_t index = node.key;
      if (node.key >= num_poses) {
        prefix = robot_id_to_vertex_prefix.at(robot_id_);
        index = node.key - num_poses;  // account for offset
      }
      gtsam::Symbol key = gtsam::Symbol(prefix, index);
      gtsam::Pose3 pose = RosToGtsam(node.pose);
      insertDpgmoValues(key, pose);
    }

    // Update optimized path
    *optimized_path_ = getOptimizedTrajectory(robot_id_);
  }  // end interface critical section
}

void KimeraPgmo::publishTransforms() {
  if (optimized_path_->size() == 0) return;

  const gtsam::Pose3& latest_pose = optimized_path_->at(optimized_path_->size() - 1);
  const gtsam::Point3& pos = latest_pose.translation();
  const gtsam::Quaternion& quat = latest_pose.rotation().toQuaternion();
  // Create transfomr message

  geometry_msgs::TransformStamped transform;
  std::string frame_name = "pgmo_base_link_";
  transform.header.stamp = timestamps_.back();
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

bool KimeraPgmo::saveMeshCallback(std_srvs::Empty::Request&,
                                  std_srvs::Empty::Response&) {
  // Save mesh
  std::string ply_name = config_.log_path + std::string("/mesh_pgmo.ply");
  WriteMeshWithStampsToPly(ply_name, *optimized_mesh_, mesh_vertex_stamps_);
  ROS_INFO("KimeraPgmo: Saved mesh to file.");
  return true;
}

bool KimeraPgmo::saveTrajectoryCallback(std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&) {
  // Save trajectory
  std::string csv_name = config_.log_path + std::string("/traj_pgmo.csv");
  saveTrajectory(*optimized_path_, timestamps_, csv_name);
  ROS_INFO("KimeraPgmo: Saved trajectories to file.");
  return true;
}

bool KimeraPgmo::saveGraphCallback(std_srvs::Empty::Request&,
                                   std_srvs::Empty::Response&) {
  // Save trajectory
  std::ofstream csvfile;
  std::string dgrf_name = config_.log_path + std::string("/pgmo.dgrf");
  saveDeformationGraph(dgrf_name);
  std::string sparse_mapping_name =
      config_.log_path + std::string("/sparsification_mapping.txt");
  savePoseGraphSparseMapping(sparse_mapping_name);
  ROS_INFO("KimeraPgmo: Saved deformation graph to file.");
  return true;
}

bool KimeraPgmo::loadGraphMeshCallback(kimera_pgmo::LoadGraphMesh::Request& request,
                                       kimera_pgmo::LoadGraphMesh::Response& response) {
  ROS_INFO("Loading deformation graph file: %s and ply file: %s. ",
           request.dgrf_file.c_str(),
           request.ply_file.c_str());
  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    response.success = loadGraphAndMesh(request.robot_id,
                                        request.ply_file,
                                        request.dgrf_file,
                                        request.sparse_mapping_file,
                                        optimized_mesh_,
                                        &mesh_vertex_stamps_,
                                        true);
  }  // end interface critical section
  if (response.success && optimized_mesh_pub_.getNumSubscribers() > 0) {
    std_msgs::Header msg_header;
    msg_header.frame_id = frame_id_;
    msg_header.stamp = ros::Time::now();
    publishMesh(*optimized_mesh_, msg_header, &optimized_mesh_pub_);
  }
  return response.success;
}

bool KimeraPgmo::requestMeshEdgesCallback(
    kimera_pgmo::RequestMeshFactors::Request& request,
    kimera_pgmo::RequestMeshFactors::Response& response) {
  size_t offset_vertex_indices = 0;
  if (request.reindex_vertices) offset_vertex_indices = trajectory_.size();
  if (getConsistencyFactors(
          request.robot_id, &response.mesh_factors, offset_vertex_indices)) {
    dpgmo_num_poses_last_req_.push(trajectory_.size());
    return true;
  }
  return false;
}

void KimeraPgmo::logStats(const std::string filename) const {
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
  size_t num_keyframes = trajectory_.size();
  // Number of vertices (total)
  size_t num_vertices = optimized_mesh_->cloud.width * optimized_mesh_->cloud.height;

  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << 1 << "," << num_keyframes << "," << num_loop_closures_ << ","
       << deformation_graph_->getGtsamFactors().size() << "," << num_vertices << ","
       << deformation_graph_->getNumVertices() << "," << inc_mesh_cb_time_ << ","
       << full_mesh_cb_time_ << "," << pg_cb_time_ << "," << path_cb_time_ << std::endl;
  file.close();
}

}  // namespace kimera_pgmo
