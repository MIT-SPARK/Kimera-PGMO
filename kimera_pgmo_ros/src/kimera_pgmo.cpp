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
  field(config.output_prefix, "output_prefix");
  field(config.log_output, "log_output");
  checkCondition(robot_id_to_prefix.count(config.robot_id), "robot_id");
}

// Constructor
KimeraPgmo::KimeraPgmo(const rclcpp::NodeOptions& options)
    : KimeraPgmoInterface(config::fromContext<KimeraPgmo::Config>()),
      rclcpp::Node("kimera_pgmo_node", options),
      config_(config::fromContext<KimeraPgmo::Config>()),
      optimized_mesh_(new pcl::PolygonMesh),
      optimized_path_(new Path),
      inc_mesh_cb_time_(0),
      full_mesh_cb_time_(0),
      pg_cb_time_(0),
      path_cb_time_(0) {
  logging::Logger::addSink("ros", std::make_shared<RosLogSink>(get_logger()));
  if (config_.log_path != "") {
    RCLCPP_INFO_STREAM(get_logger(),
                       "Saving optimized data to: "
                           << config_.log_path << "/ mesh_pgmo.ply and traj_pgmo.csv");
    if (config_.log_output) {
      RCLCPP_INFO_STREAM(
          get_logger(),
          "Logging output to: " << config_.log_path << "/kimera_pgmo_log.csv");
    }
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

  // Log header to file
  if (config_.log_output) {
    std::string log_file = config_.log_path + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  // Start graph thread
  graph_thread_.reset(new std::thread(&KimeraPgmo::startGraphProcess, this));
  // Start full mesh thread
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

// Initialize callbacks
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

  // Initialize save trajectory service
  save_traj_srv_ = create_service<Empty>(
      "save_trajectory", std::bind(&KimeraPgmo::saveTrajectoryCallback, this, _1, _2));

  // Initialize save deformation graph service
  save_graph_srv_ = create_service<Empty>(
      "save_dgrf", std::bind(&KimeraPgmo::saveGraphCallback, this, _1, _2));

  // Initialize save deformation graph service
  load_graph_mesh_srv_ = create_service<LoadGraphMesh>(
      "load_graph_mesh", std::bind(&KimeraPgmo::loadGraphMeshCallback, this, _1, _2));

  // Reset the deformation graph service
  reset_srv_ = create_service<Empty>(
      "reset_graph", std::bind(&KimeraPgmo::resetGraphCallback, this, _1, _2));
}

// Initialize callbacks
void KimeraPgmo::startMeshProcess() {
  using namespace std::placeholders;
  full_mesh_sub_ = create_subscription<Mesh>(
      "full_mesh", 1, std::bind(&KimeraPgmo::fullMeshCallback, this, _1));

  // Initialize save mesh service
  save_mesh_srv_ = create_service<Empty>(
      "save_mesh", std::bind(&KimeraPgmo::saveMeshCallback, this, _1, _2));
}

// To publish optimized mesh
bool KimeraPgmo::publishOptimizedMesh() const {
  // TODO(nathan) unused header
  std_msgs::msg::Header msg_header;
  msg_header.stamp = rclcpp::Time(last_mesh_stamp_);
  msg_header.frame_id = config_.frame_id;
  publishMesh(*optimized_mesh_);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmo::publishOptimizedPath() const {
  if (optimized_path_->size() == 0) {
    return false;
  }

  std_msgs::msg::Header msg_header;
  msg_header.stamp = rclcpp::Time(timestamps_.back());
  msg_header.frame_id = config_.frame_id;
  publishPath(*optimized_path_, msg_header, *optimized_path_pub_);

  if (optimized_odom_pub_->get_subscription_count() > 0) {
    // Publish also the optimized odometry
    nav_msgs::msg::Odometry odometry_msg;
    const gtsam::Pose3 last_pose = optimized_path_->at(optimized_path_->size() - 1);
    const gtsam::Rot3& rotation = last_pose.rotation();
    const gtsam::Quaternion& quaternion = rotation.toQuaternion();

    // Create header.
    odometry_msg.header.stamp = rclcpp::Time(timestamps_.back());
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

    optimized_odom_pub_->publish(odometry_msg);
  }

  return true;
}

void KimeraPgmo::publishTransforms() {
  if (optimized_path_->size() == 0) return;

  const gtsam::Pose3& latest_pose = optimized_path_->at(optimized_path_->size() - 1);
  const gtsam::Point3& pos = latest_pose.translation();
  const gtsam::Quaternion& quat = latest_pose.rotation().toQuaternion();
  // Create transform message

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

  // Start timer
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

  if (pose_graph_pub_->get_subscription_count() > 0) {
    // Publish pose graph
    std::map<size_t, std::vector<Timestamp>> id_timestamps;
    id_timestamps[config_.robot_id] = timestamps_;
    const auto pose_graph_ptr = deformation_graph_->getPoseGraph(id_timestamps);
    pose_graph_pub_->publish(*pose_graph_ptr);
  }

  // Publish optimized trajectory
  publishOptimizedPath();
}

void KimeraPgmo::optimizedPathCallback(const nav_msgs::msg::Path& msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  Path path;
  for (const auto& stamped_pose : msg.poses) {
    auto& pose = path.emplace_back();
    conversions::PoseTypeAdapter::convert_to_custom(stamped_pose.pose, pose);
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

void KimeraPgmo::fullMeshCallback(const Mesh& msg) {
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

  if (opt_mesh && optimized_mesh_pub_->get_subscription_count() > 0) {
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

void KimeraPgmo::incrementalMeshGraphCallback(const PoseGraph& graph) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    // TODO(Yun) maybe just sync mesh and pose graph callback
    processIncrementalMeshGraph(
        graph, timestamps_, unconnected_mesh_indices_, unconnected_mesh_index_stamps_);
  }  // end interface critical section

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  inc_mesh_cb_time_ = spin_duration.count();

  return;
}

void KimeraPgmo::saveMeshCallback(const Empty::Request::SharedPtr&,
                                  Empty::Response::SharedPtr) {
  // Save mesh
  std::string ply_name = config_.log_path + std::string("/mesh_pgmo.ply");
  WriteMeshWithStampsToPly(ply_name, *optimized_mesh_, mesh_vertex_stamps_);
  RCLCPP_INFO(get_logger(), "KimeraPgmo: Saved mesh to file.");
}

void KimeraPgmo::saveTrajectoryCallback(const Empty::Request::SharedPtr&,
                                        Empty::Response::SharedPtr) {
  // Save trajectory
  std::string csv_name = config_.log_path + std::string("/traj_pgmo.csv");
  saveTrajectory(*optimized_path_, timestamps_, csv_name);
  RCLCPP_INFO(get_logger(), "KimeraPgmo: Saved trajectories to file.");
}

void KimeraPgmo::saveGraphCallback(const Empty::Request::SharedPtr&,
                                   Empty::Response::SharedPtr) {
  // Save trajectory
  std::string dgrf_name = config_.log_path + std::string("/pgmo.dgrf");
  saveDeformationGraph(dgrf_name);
  RCLCPP_INFO(get_logger(), "KimeraPgmo: Saved deformation graph to file.");
}

void KimeraPgmo::loadGraphMeshCallback(const LoadGraphSrv::Request::SharedPtr& request,
                                       LoadGraphSrv::Response::SharedPtr response) {
  RCLCPP_INFO(get_logger(),
              "Loading deformation graph file: %s and ply file: %s. ",
              request->dgrf_file.c_str(),
              request->ply_file.c_str());

  {  // start interface critical section
    std::unique_lock<std::mutex> lock(interface_mutex_);
    response->success = loadGraphAndMesh(request->robot_id,
                                         request->ply_file,
                                         request->dgrf_file,
                                         optimized_mesh_,
                                         &mesh_vertex_stamps_,
                                         true);
  }  // end interface critical section

  if (response->success && optimized_mesh_pub_->get_subscription_count() > 0) {
    std_msgs::msg::Header msg_header;
    msg_header.frame_id = config_.frame_id;
    // TODO(nathan) get node clock
    msg_header.stamp = rclcpp::Clock().now();
    publishMesh(*optimized_mesh_);
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

  // Publish the msg with the edges
  viz_mesh_mesh_edges_pub_->publish(mesh_mesh_viz);
  viz_pose_mesh_edges_pub_->publish(pose_mesh_viz);
}

void KimeraPgmo::logStats(const std::string& filename) const {
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
       << deformation_graph_->getFactors()->size() << "," << num_vertices << ","
       << deformation_graph_->getNumVertices() << "," << inc_mesh_cb_time_ << ","
       << full_mesh_cb_time_ << "," << pg_cb_time_ << "," << path_cb_time_ << std::endl;
  file.close();
}

bool KimeraPgmo::publishMesh(const pcl::PolygonMesh& mesh) const {
  if (optimized_mesh_pub_->get_subscription_count() == 0) {
    return false;
  }

  if (mesh.cloud.height * mesh.cloud.width == 0) {
    return false;
  }

  auto msg =
      conversions::toMsg(config_.robot_id, mesh, mesh_vertex_stamps_, config_.frame_id);
  optimized_mesh_pub_->publish(std::move(msg));
  return true;
}

bool KimeraPgmo::publishPath(const Path& path,
                             const std_msgs::msg::Header& header,
                             rclcpp::Publisher<nav_msgs::msg::Path>& pub) const {
  if (path.size() == 0) {
    return false;
  }

  if (pub.get_subscription_count() == 0) {
    return false;
  }

  // Create message type
  auto path_msg = std::make_unique<nav_msgs::msg::Path>();
  path_msg->poses.reserve(path.size());
  for (size_t i = 0; i < path.size(); i++) {
    gtsam::Pose3 pose = path.at(i);
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

  // Publish path message
  path_msg->header = header;
  pub.publish(std::move(path_msg));
  return true;
}

void KimeraPgmo::resetGraphCallback(const Empty::Request::SharedPtr&,
                                    Empty::Response::SharedPtr) {
  resetDeformationGraph();
}

}  // namespace kimera_pgmo
