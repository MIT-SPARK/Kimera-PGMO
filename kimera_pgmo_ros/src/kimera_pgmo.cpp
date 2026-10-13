/**
 * @file   kimera_pgmo.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/kimera_pgmo.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <kimera_pgmo/utils/logging.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <pose_graph_tools_ros/conversions.h>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

#include "kimera_pgmo_ros/conversion/gtsam.h"
#include "kimera_pgmo_ros/conversion/mesh.h"
#include "kimera_pgmo_ros/ros_log_sink.h"
#include "kimera_pgmo_ros/visualization_functions.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::msg::Mesh;
using kimera_pgmo_msgs::srv::LoadGraphMesh;
using nav_msgs::msg::Odometry;
using pose_graph_tools::PoseGraph;
using std_srvs::srv::Empty;
using visualization_msgs::msg::Marker;

using PoseGraphAdapter = pose_graph_tools::PoseGraphTypeAdapter;

void declare_config(KimeraPgmo::Config& config) {
  using namespace config;
  name("KimeraPgmo::Config");
  base<KimeraPgmoConfig>(config);
  field(config.frame_id, "frame_id");
  field(config.robot_id, "robot_id");
  checkCondition(robot_id_to_prefix.count(config.robot_id), "robot_id");
}

KimeraPgmo::KimeraPgmo(const rclcpp::NodeOptions& options)
    : KimeraPgmoInterface(config::fromContext<KimeraPgmo::Config>()),
      rclcpp::Node("kimera_pgmo_node", options),
      config_(config::fromContext<KimeraPgmo::Config>()),
      inc_mesh_cb_time_(0),
      full_mesh_cb_time_(0),
      pg_cb_time_(0),
      path_cb_time_(0) {
  logging::Logger::addSink("ros", std::make_shared<RosLogSink>(get_logger()));
  if (config_.log_path != "") {
    RCLCPP_INFO_STREAM(get_logger(),
                       "Saving optimized data to: "
                           << config_.log_path << "/ mesh_pgmo.ply and traj_pgmo.csv");
  }

  optimized_mesh_pub_ = create_publisher<Mesh>("optimized_mesh", 1);
  optimized_odom_pub_ = create_publisher<Odometry>("optimized_odom", 1);
  pose_graph_pub_ = create_publisher<PoseGraphAdapter>("pose_graph", 1);
  optimized_path_pub_ = create_publisher<nav_msgs::msg::Path>("optimized_path", 1);
  viz_mesh_mesh_edges_pub_ =
      create_publisher<Marker>("deformation_graph_mesh_mesh", 10);
  viz_pose_mesh_edges_pub_ =
      create_publisher<Marker>("deformation_graph_pose_mesh", 10);

  tf_broadcast_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

  graph_thread_.reset(new std::thread(&KimeraPgmo::startGraphProcess, this));
  mesh_thread_.reset(new std::thread(&KimeraPgmo::startMeshProcess, this));

  RCLCPP_INFO(get_logger(), "Initialized Kimera-PGMO.");
}

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

void KimeraPgmo::startGraphProcess() {
  using namespace std::placeholders;
  incremental_mesh_graph_sub_ = create_subscription<PoseGraphAdapter>(
      "mesh_graph_incremental",
      5000,
      std::bind(&KimeraPgmo::incrementalMeshGraphCallback, this, _1));

  pose_graph_incremental_sub_ = create_subscription<PoseGraphAdapter>(
      "pose_graph_incremental",
      5000,
      std::bind(&KimeraPgmo::incrementalPoseGraphCallback, this, _1));

  path_callback_sub_ = create_subscription<nav_msgs::msg::Path>(
      "input_path", 2, std::bind(&KimeraPgmo::optimizedPathCallback, this, _1));

  save_traj_srv_ = create_service<Empty>(
      "save_trajectory", std::bind(&KimeraPgmo::saveTrajectoryCallback, this, _1, _2));

  save_graph_srv_ = create_service<Empty>(
      "save_dgrf", std::bind(&KimeraPgmo::saveGraphCallback, this, _1, _2));

  load_graph_mesh_srv_ = create_service<LoadGraphMesh>(
      "load_graph_mesh", std::bind(&KimeraPgmo::loadGraphMeshCallback, this, _1, _2));

  reset_srv_ = create_service<Empty>(
      "reset_graph", std::bind(&KimeraPgmo::resetGraphCallback, this, _1, _2));
}

void KimeraPgmo::startMeshProcess() {
  using namespace std::placeholders;
  full_mesh_sub_ = create_subscription<Mesh>(
      "full_mesh", 1, std::bind(&KimeraPgmo::fullMeshCallback, this, _1));

  save_mesh_srv_ = create_service<Empty>(
      "save_mesh", std::bind(&KimeraPgmo::saveMeshCallback, this, _1, _2));
}

bool KimeraPgmo::publishOptimizedMesh() const {
  publishMesh();
  return true;
}

bool KimeraPgmo::publishOptimizedPath() const {
  if (optimized_path_.empty()) {
    return false;
  }

  std_msgs::msg::Header msg_header;
  msg_header.stamp = rclcpp::Time(timestamps_.back());
  msg_header.frame_id = config_.frame_id;
  publishPath(msg_header, *optimized_path_pub_);

  if (optimized_odom_pub_->get_subscription_count() > 0) {
    nav_msgs::msg::Odometry odometry_msg;
    const gtsam::Pose3 last_pose = optimized_path_.back();
    const auto quaternion = last_pose.rotation().toQuaternion();

    odometry_msg.header.stamp = rclcpp::Time(timestamps_.back());
    odometry_msg.header.frame_id = config_.frame_id;
    odometry_msg.pose.pose.position.x = last_pose.x();
    odometry_msg.pose.pose.position.y = last_pose.y();
    odometry_msg.pose.pose.position.z = last_pose.z();
    odometry_msg.pose.pose.orientation.w = quaternion.w();
    odometry_msg.pose.pose.orientation.x = quaternion.x();
    odometry_msg.pose.pose.orientation.y = quaternion.y();
    odometry_msg.pose.pose.orientation.z = quaternion.z();
    optimized_odom_pub_->publish(odometry_msg);
  }

  return true;
}

void KimeraPgmo::publishTransforms() {
  if (optimized_path_.size() == 0) {
    return;
  }

  const gtsam::Pose3& latest_pose = optimized_path_.back();
  const gtsam::Point3& pos = latest_pose.translation();
  const gtsam::Quaternion& quat = latest_pose.rotation().toQuaternion();

  geometry_msgs::msg::TransformStamped transform;
  std::string frame_name = "pgmo_base_link_";
  transform.header.stamp = rclcpp::Time(timestamps_.back());
  transform.header.frame_id = "world";
  transform.child_frame_id = frame_name;
  transform.transform.translation.x = pos.x();
  transform.transform.translation.y = pos.y();
  transform.transform.translation.z = pos.z();
  transform.transform.rotation.x = quat.x();
  transform.transform.rotation.y = quat.y();
  transform.transform.rotation.z = quat.z();
  transform.transform.rotation.w = quat.w();
  tf_broadcast_->sendTransform(transform);
}

void KimeraPgmo::incrementalPoseGraphCallback(const PoseGraph& graph) {
  if (graph.nodes.size() == 0 && graph.edges.size() == 0) {
    return;
  }

  auto start = std::chrono::high_resolution_clock::now();

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    processIncrementalPoseGraph(graph,
                                unconnected_mesh_indices_,
                                unconnected_mesh_index_stamps_,
                                trajectory_,
                                timestamps_);
    unconnected_mesh_indices_.clear();
    unconnected_mesh_index_stamps_.clear();
    optimized_path_ = getOptimizedTrajectory(config_.robot_id);
  }  // end interface critical section

  publishTransforms();

  auto stop = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  pg_cb_time_ = elapsed.count();

  if (pose_graph_pub_->get_subscription_count() > 0) {
    const auto pose_graph_ptr = deformation_graph_->getPoseGraph();
    pose_graph_pub_->publish(*pose_graph_ptr);
  }

  publishOptimizedPath();
}

void KimeraPgmo::optimizedPathCallback(const nav_msgs::msg::Path& msg) {
  auto start = std::chrono::high_resolution_clock::now();

  std::vector<gtsam::Pose3> path;
  for (const auto& stamped_pose : msg.poses) {
    auto& pose = path.emplace_back();
    conversions::PoseTypeAdapter::convert_to_custom(stamped_pose.pose, pose);
  }

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    processOptimizedPath(path, config_.robot_id);
  }  // end interface critical section

  auto stop = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  path_cb_time_ = elapsed.count();
}

void KimeraPgmo::fullMeshCallback(const Mesh& msg) {
  auto start = std::chrono::high_resolution_clock::now();
  conversions::fillFromMsg(msg, original_vertices_, faces_);

  // Optimization always enabled
  bool opt_mesh;
  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    opt_mesh = deformFullMesh(config_.robot_id, true);
  }  // end interface critical section

  if (opt_mesh && optimized_mesh_pub_->get_subscription_count() > 0) {
    publishMesh();
  }

  auto stop = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  full_mesh_cb_time_ = elapsed.count();

  visualizeDeformationGraph();
}

void KimeraPgmo::incrementalMeshGraphCallback(const PoseGraph& graph) {
  auto start = std::chrono::high_resolution_clock::now();

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    // TODO(Yun) maybe just sync mesh and pose graph callback
    processIncrementalMeshGraph(
        graph, timestamps_, unconnected_mesh_indices_, unconnected_mesh_index_stamps_);
  }  // end interface critical section

  auto stop = std::chrono::high_resolution_clock::now();
  auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  inc_mesh_cb_time_ = elapsed.count();
}

void KimeraPgmo::saveMeshCallback(const Empty::Request::SharedPtr&,
                                  Empty::Response::SharedPtr) {
  saveMesh(config_.log_path + std::string("/mesh_pgmo.ply"));
  RCLCPP_INFO(get_logger(), "KimeraPgmo: Saved mesh to file.");
}

void KimeraPgmo::saveTrajectoryCallback(const Empty::Request::SharedPtr&,
                                        Empty::Response::SharedPtr) {
  saveTrajectory(config_.log_path + std::string("/traj_pgmo.csv"));
  RCLCPP_INFO(get_logger(), "KimeraPgmo: Saved trajectories to file.");
}

void KimeraPgmo::saveGraphCallback(const Empty::Request::SharedPtr&,
                                   Empty::Response::SharedPtr) {
  saveDeformationGraph(config_.log_path + std::string("/pgmo.json"));
  RCLCPP_INFO(get_logger(), "KimeraPgmo: Saved deformation graph to file.");
}

void KimeraPgmo::loadGraphMeshCallback(const LoadGraphSrv::Request::SharedPtr& req,
                                       LoadGraphSrv::Response::SharedPtr rep) {
  RCLCPP_INFO(get_logger(),
              "Loading deformation graph file: %s and ply file: %s. ",
              req->dgrf_file.c_str(),
              req->ply_file.c_str());

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    rep->success = loadGraphAndMesh(req->robot_id, req->ply_file, req->dgrf_file, true);
  }  // end interface critical section

  if (rep->success && optimized_mesh_pub_->get_subscription_count() > 0) {
    publishMesh();
  }
}

void KimeraPgmo::visualizeDeformationGraph() const {
  if (viz_mesh_mesh_edges_pub_->get_subscription_count() == 0 &&
      viz_pose_mesh_edges_pub_->get_subscription_count() == 0) {
    return;
  }

  // TODO(nathan) get actual node clock
  const auto curr_time = rclcpp::Clock().now();
  visualization_msgs::msg::Marker mesh_mesh_viz;
  visualization_msgs::msg::Marker pose_mesh_viz;
  fillDeformationGraphMarkers(
      *deformation_graph_, curr_time, mesh_mesh_viz, pose_mesh_viz);

  viz_mesh_mesh_edges_pub_->publish(mesh_mesh_viz);
  viz_pose_mesh_edges_pub_->publish(pose_mesh_viz);
}

bool KimeraPgmo::publishMesh() const {
  if (optimized_mesh_pub_->get_subscription_count() == 0) {
    return false;
  }

  if (optimized_vertices_.empty()) {
    return false;
  }

  auto msg = conversions::toMsg(config_.robot_id, optimized_vertices_, faces_);
  msg->header.stamp = rclcpp::Time(last_mesh_stamp_);
  msg->header.frame_id = config_.frame_id;
  optimized_mesh_pub_->publish(std::move(msg));
  return true;
}

bool KimeraPgmo::publishPath(const std_msgs::msg::Header& header,
                             rclcpp::Publisher<nav_msgs::msg::Path>& pub) const {
  if (optimized_path_.size() == 0) {
    return false;
  }

  if (pub.get_subscription_count() == 0) {
    return false;
  }

  auto path_msg = std::make_unique<nav_msgs::msg::Path>();
  path_msg->poses.reserve(optimized_path_.size());
  for (size_t i = 0; i < optimized_path_.size(); i++) {
    gtsam::Pose3 pose = optimized_path_.at(i);
    gtsam::Point3 trans = pose.translation();
    gtsam::Quaternion quat = pose.rotation().toQuaternion();

    geometry_msgs::msg::PoseStamped ps_msg;
    ps_msg.header.frame_id = header.frame_id;
    ps_msg.pose.position.x = trans.x();
    ps_msg.pose.position.y = trans.y();
    ps_msg.pose.position.z = trans.z();
    ps_msg.pose.orientation.x = quat.x();
    ps_msg.pose.orientation.y = quat.y();
    ps_msg.pose.orientation.z = quat.z();
    ps_msg.pose.orientation.w = quat.w();

    path_msg->poses.push_back(ps_msg);
  }

  path_msg->header = header;
  pub.publish(std::move(path_msg));
  return true;
}

void KimeraPgmo::resetGraphCallback(const Empty::Request::SharedPtr&,
                                    Empty::Response::SharedPtr) {
  resetDeformationGraph();
}

}  // namespace kimera_pgmo
