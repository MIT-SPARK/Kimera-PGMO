/**
 * @file   kimera_pgmo.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/kimera_pgmo.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <kimera_pgmo/utils/logging.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <nav_msgs/Odometry.h>
#include <pose_graph_tools_ros/conversions.h>
#include <visualization_msgs/Marker.h>

#include "kimera_pgmo_ros/conversion/gtsam_conversions.h"
#include "kimera_pgmo_ros/conversion/mesh_conversion.h"
#include "kimera_pgmo_ros/visualization_functions.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::KimeraPgmoMesh;
using kimera_pgmo_msgs::LoadGraphMesh;
using kimera_pgmo_msgs::RequestMeshFactors;
using pose_graph_tools_msgs::PoseGraph;
using visualization_msgs::Marker;

void declare_config(KimeraPgmo::Config& config) {
  using namespace config;
  name("KimeraPgmo::Config");
  base<KimeraPgmoConfig>(config);
  field(config.frame_id, "frame_id");
  field(config.robot_id, "robot_id");
  field(config.output_prefix, "output_prefix");
  field(config.log_output, "log_output");
  checkCondition(robot_id_to_prefix.count(config.robot_id), "robot_id");
}

// Constructor
KimeraPgmo::KimeraPgmo()
    : optimized_mesh_(new pcl::PolygonMesh),
      optimized_path_(new Path),
      inc_mesh_cb_time_(0),
      full_mesh_cb_time_(0),
      pg_cb_time_(0),
      path_cb_time_(0) {}

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
bool KimeraPgmo::initFromRos(const ros::NodeHandle& n) {
  nh_ = n;
  config_ = config::fromRos<KimeraPgmo::Config>(nh_);
  initialize(config_);

  if (config_.log_path != "") {
    ROS_INFO_STREAM("Saving optimized data to: "
                    << config_.log_path << "/ mesh_pgmo.ply and traj_pgmo.csv");
    if (config_.log_output) {
      ROS_INFO_STREAM("Logging output to: " << config_.log_path
                                            << "/kimera_pgmo_log.csv");
    }
  }

  optimized_mesh_pub_ = nh_.advertise<KimeraPgmoMesh>("optimized_mesh", 1, false);
  optimized_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("optimized_odom", 1, false);
  pose_graph_pub_ = nh_.advertise<PoseGraph>("pose_graph", 1, false);
  optimized_path_pub_ = nh_.advertise<nav_msgs::Path>("optimized_path", 1, false);
  viz_mesh_mesh_edges_pub_ =
      nh_.advertise<Marker>("deformation_graph_mesh_mesh", 10, false);
  viz_pose_mesh_edges_pub_ =
      nh_.advertise<Marker>("deformation_graph_pose_mesh", 10, false);

  // Log header to file
  if (config_.log_output) {
    std::string log_file = config_.log_path + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  // Start graph thread
  graph_thread_.reset(new std::thread(&KimeraPgmo::startGraphProcess, this));
  // Start full mesh thread
  mesh_thread_.reset(new std::thread(&KimeraPgmo::startMeshProcess, this));

  ROS_INFO("Initialized Kimera-PGMO.");
  return true;
}

// Initialize callbacks
void KimeraPgmo::startGraphProcess() {
  incremental_mesh_graph_sub_ = nh_.subscribe(
      "mesh_graph_incremental", 5000, &KimeraPgmo::incrementalMeshGraphCallback, this);

  pose_graph_incremental_sub_ = nh_.subscribe(
      "pose_graph_incremental", 5000, &KimeraPgmo::incrementalPoseGraphCallback, this);

  path_callback_sub_ =
      nh_.subscribe("input_path", 2, &KimeraPgmo::optimizedPathCallback, this);

  dpgmo_callback_sub_ =
      nh_.subscribe("optimized_values", 1, &KimeraPgmo::dpgmoCallback, this);

  // Initialize save trajectory service
  save_traj_srv_ = nh_.advertiseService(
      "save_trajectory", &KimeraPgmo::saveTrajectoryCallback, this);

  // Initialize save deformation graph service
  save_graph_srv_ =
      nh_.advertiseService("save_dgrf", &KimeraPgmo::saveGraphCallback, this);

  // Initialize save deformation graph service
  load_graph_mesh_srv_ =
      nh_.advertiseService("load_graph_mesh", &KimeraPgmo::loadGraphMeshCallback, this);

  // Reset the deformation graph service
  reset_srv_ =
      nh_.advertiseService("reset_graph", &KimeraPgmo::resetGraphCallback, this);

  // Initialize request mesh edges service
  req_mesh_edges_srv_ = nh_.advertiseService(
      "get_mesh_edges", &KimeraPgmo::requestMeshEdgesCallback, this);
}

// Initialize callbacks
void KimeraPgmo::startMeshProcess() {
  full_mesh_sub_ = nh_.subscribe("full_mesh", 1, &KimeraPgmo::fullMeshCallback, this);

  // Initialize save mesh service
  save_mesh_srv_ =
      nh_.advertiseService("save_mesh", &KimeraPgmo::saveMeshCallback, this);
}

// To publish optimized mesh
bool KimeraPgmo::publishOptimizedMesh() const {
  std_msgs::Header msg_header;
  msg_header.stamp = last_mesh_stamp_;
  msg_header.frame_id = config_.frame_id;
  publishMesh(*optimized_mesh_);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmo::publishOptimizedPath() const {
  if (optimized_path_->size() == 0) {
    return false;
  }

  std_msgs::Header msg_header;
  msg_header.stamp.fromNSec(timestamps_.back());
  msg_header.frame_id = config_.frame_id;
  publishPath(*optimized_path_, msg_header, &optimized_path_pub_);

  if (optimized_odom_pub_.getNumSubscribers() > 0) {
    // Publish also the optimized odometry
    nav_msgs::Odometry odometry_msg;
    const gtsam::Pose3 last_pose = optimized_path_->at(optimized_path_->size() - 1);
    const gtsam::Rot3& rotation = last_pose.rotation();
    const gtsam::Quaternion& quaternion = rotation.toQuaternion();

    // Create header.
    odometry_msg.header.stamp.fromNSec(timestamps_.back());
    odometry_msg.header.frame_id = config_.frame_id;

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

void KimeraPgmo::publishTransforms() {
  if (optimized_path_->size() == 0) return;

  const gtsam::Pose3& latest_pose = optimized_path_->at(optimized_path_->size() - 1);
  const gtsam::Point3& pos = latest_pose.translation();
  const gtsam::Quaternion& quat = latest_pose.rotation().toQuaternion();
  // Create transfomr message

  geometry_msgs::TransformStamped transform;
  std::string frame_name = "pgmo_base_link_";
  transform.header.stamp.fromNSec(timestamps_.back());
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

void KimeraPgmo::incrementalPoseGraphCallback(const PoseGraph& msg) {
  if (msg.nodes.size() == 0 && msg.edges.size() == 0) {
    return;
  }

  const auto graph = pose_graph_tools::fromMsg(msg);

  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    processIncrementalPoseGraph(graph, trajectory_, timestamps_, unconnected_nodes_);
    // Update optimized path
    *optimized_path_ = getOptimizedTrajectory(config_.robot_id);
  }  // end interface critical section

  // Update transforms
  publishTransforms();

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  pg_cb_time_ = spin_duration.count();

  // Log to file
  if (config_.log_output) {
    std::string log_file = config_.log_path + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    // Publish pose graph
    std::map<size_t, std::vector<Timestamp>> id_timestamps;
    id_timestamps[config_.robot_id] = timestamps_;
    const auto pose_graph_ptr = deformation_graph_->getPoseGraph(id_timestamps);
    const auto msg = pose_graph_tools::toMsg(*pose_graph_ptr);
    pose_graph_pub_.publish(msg);
  }

  // Publish optimized trajectory
  publishOptimizedPath();
}

void KimeraPgmo::optimizedPathCallback(const nav_msgs::Path& msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  Path path;
  for (const auto& stamped_pose : msg.poses) {
    path.push_back(conversions::RosToGtsam(stamped_pose.pose));
  }

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    processOptimizedPath(path, config_.robot_id);
  }  // end interface critical section

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  path_cb_time_ = spin_duration.count();

  // Log to file
  if (config_.log_output) {
    std::string log_file = config_.log_path + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }
}

void KimeraPgmo::fullMeshCallback(const kimera_pgmo_msgs::KimeraPgmoMesh& msg) {
  auto start = std::chrono::high_resolution_clock::now();

  std::vector<int> graph_indices;
  auto mesh = conversions::fromMsg(msg, &mesh_vertex_stamps_, &graph_indices);

  bool opt_mesh;
  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    // Optimization always happen here only to ensure that the full mesh is
    // always optimized when published
    opt_mesh = optimizeFullMesh(config_.robot_id,
                                mesh,
                                mesh_vertex_stamps_,
                                graph_indices,
                                *optimized_mesh_,
                                true);
  }  // end interface critical section

  if (opt_mesh && optimized_mesh_pub_.getNumSubscribers() > 0) {
    publishMesh(*optimized_mesh_);
  }

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  full_mesh_cb_time_ = spin_duration.count();

  // Publish deformation graph edges visualization
  visualizeDeformationGraph();
  return;
}

void KimeraPgmo::incrementalMeshGraphCallback(const PoseGraph& msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();
  const auto graph = pose_graph_tools::fromMsg(msg);

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    processIncrementalMeshGraph(graph, timestamps_, unconnected_nodes_);
  }  // end interface critical section

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  inc_mesh_cb_time_ = spin_duration.count();

  return;
}

void KimeraPgmo::dpgmoCallback(const pose_graph_tools_msgs::PoseGraph& msg) {
  if (dpgmo_num_poses_last_req_.empty()) {
    ROS_ERROR("Mesh factors request queue empty.");
    return;
  }

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    size_t num_poses = dpgmo_num_poses_last_req_.front();
    dpgmo_num_poses_last_req_.pop();
    for (const auto& node : msg.nodes) {
      if (node.robot_id != config_.robot_id) {
        ROS_WARN("Unexpected robot id in pose graph received in dpgmo callback.");
        continue;
      }

      char prefix = robot_id_to_prefix.at(config_.robot_id);
      size_t index = node.key;
      if (node.key >= num_poses) {
        prefix = robot_id_to_vertex_prefix.at(config_.robot_id);
        index = node.key - num_poses;  // account for offset
      }

      gtsam::Symbol key = gtsam::Symbol(prefix, index);
      gtsam::Pose3 pose = conversions::RosToGtsam(node.pose);
      insertDpgmoValues(key, pose);
    }

    // Update optimized path
    *optimized_path_ = getOptimizedTrajectory(config_.robot_id);
  }  // end interface critical section
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

bool KimeraPgmo::loadGraphMeshCallback(LoadGraphMesh::Request& request,
                                       LoadGraphMesh::Response& response) {
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
    msg_header.frame_id = config_.frame_id;
    msg_header.stamp = ros::Time::now();
    publishMesh(*optimized_mesh_);
  }

  return response.success;
}

bool KimeraPgmo::requestMeshEdgesCallback(RequestMeshFactors::Request& request,
                                          RequestMeshFactors::Response& response) {
  size_t offset_vertex_indices = 0;
  if (request.reindex_vertices) {
    offset_vertex_indices = trajectory_.size();
  }

  pose_graph_tools::PoseGraph factors;
  if (!getConsistencyFactors(request.robot_id, factors, offset_vertex_indices)) {
    return false;
  }

  response.mesh_factors = pose_graph_tools::toMsg(factors);
  dpgmo_num_poses_last_req_.push(trajectory_.size());
  return true;
}

void KimeraPgmo::visualizeDeformationGraph() const {
  if (viz_mesh_mesh_edges_pub_.getNumSubscribers() == 0 &&
      viz_pose_mesh_edges_pub_.getNumSubscribers() == 0) {
    return;
  }

  const ros::Time curr_time = ros::Time::now();
  visualization_msgs::Marker mesh_mesh_viz;
  visualization_msgs::Marker pose_mesh_viz;
  fillDeformationGraphMarkers(
      *deformation_graph_, curr_time, mesh_mesh_viz, pose_mesh_viz);

  // Publish the msg with the edges
  viz_mesh_mesh_edges_pub_.publish(mesh_mesh_viz);
  viz_pose_mesh_edges_pub_.publish(pose_mesh_viz);
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

bool KimeraPgmo::publishMesh(const pcl::PolygonMesh& mesh) const {
  if (optimized_mesh_pub_.getNumSubscribers() == 0) {
    return false;
  }

  if (mesh.cloud.height * mesh.cloud.width == 0) {
    return false;
  }

  const auto msg =
      conversions::toMsg(config_.robot_id, mesh, mesh_vertex_stamps_, config_.frame_id);
  optimized_mesh_pub_.publish(msg);
  return true;
}

bool KimeraPgmo::publishPath(const Path& path,
                             const std_msgs::Header& header,
                             const ros::Publisher* publisher) const {
  if (path.size() == 0) {
    return false;
  }

  if (publisher->getNumSubscribers() == 0) {
    return false;
  }

  // Create message type
  nav_msgs::Path path_msg;
  path_msg.poses.reserve(path.size());
  for (size_t i = 0; i < path.size(); i++) {
    gtsam::Pose3 pose = path.at(i);
    gtsam::Point3 trans = pose.translation();
    gtsam::Quaternion quat = pose.rotation().toQuaternion();

    geometry_msgs::PoseStamped ps_msg;
    ps_msg.header.frame_id = header.frame_id;
    ps_msg.pose.position.x = trans.x();
    ps_msg.pose.position.y = trans.y();
    ps_msg.pose.position.z = trans.z();
    ps_msg.pose.orientation.x = quat.x();
    ps_msg.pose.orientation.y = quat.y();
    ps_msg.pose.orientation.z = quat.z();
    ps_msg.pose.orientation.w = quat.w();

    path_msg.poses.push_back(ps_msg);
  }

  // Publish path message
  path_msg.header = header;
  publisher->publish(path_msg);

  return true;
}

}  // namespace kimera_pgmo
