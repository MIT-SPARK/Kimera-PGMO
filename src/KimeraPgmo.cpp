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
    : num_loop_closures_(0),
      inc_mesh_cb_time_(0),
      full_mesh_cb_time_(0),
      pg_cb_time_(0),
      path_cb_time_(0) {}

KimeraPgmo::~KimeraPgmo() {}

// Initialize parameters, publishers, and subscribers and deformation graph
bool KimeraPgmo::initialize(const ros::NodeHandle& n) {
  if (!loadParameters(n)) {
    ROS_ERROR("KimeraPgmo: Failed to load parameters.");
  }

  if (!createPublishers(n)) {
    ROS_ERROR("KimeraPgmo: Failed to create publishers.");
  }

  if (!registerCallbacks(n)) {
    ROS_ERROR("KimeraPgmo: Failed to register callbacks.");
  }

  // Log header to file
  if (log_output_) {
    std::string log_file = output_prefix_ + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }

  ROS_INFO("Initialized Kimera-PGMO.");

  return true;
}

// Load deformation parameters
bool KimeraPgmo::loadParameters(const ros::NodeHandle& n) {
  // Set run mode
  int run_mode_num;
  if (!n.getParam("run_mode", run_mode_num)) return false;
  run_mode_ = static_cast<RunMode>(run_mode_num);
  if (!n.getParam("use_msg_time", use_msg_time_)) return false;

  if (!n.getParam("frame_id", frame_id_)) return false;
  if (!n.getParam("compression_time_horizon", compression_time_horizon_))
    return false;
  if (n.getParam("output_prefix", output_prefix_)) {
    ROS_INFO("Saving optimized data to: %s/ mesh_pgmo.ply and traj_pgmo.csv",
             output_prefix_.c_str());
    n.getParam("log_output", log_output_);
    if (log_output_) {
      ROS_INFO("Logging output to: %s/kimera_pgmo_log.csv",
               output_prefix_.c_str());
    }
  }
  if (!n.getParam("embed_trajectory_delta_t", embed_delta_t_)) return false;

  // start the mesh compression module for deformation graph
  double deformation_graph_resolution;
  if (!n.getParam("d_graph_resolution", deformation_graph_resolution))
    return false;

  compression_.reset(new OctreeCompression(deformation_graph_resolution));

  // start deformation graph module
  double pgo_trans_threshold, pgo_rot_threshold;
  if (!n.getParam("rpgo/translation_threshold", pgo_trans_threshold))
    return false;
  if (!n.getParam("rpgo/rotation_threshold", pgo_rot_threshold)) return false;

  if (!deformation_graph_.initialize(pgo_trans_threshold, pgo_rot_threshold)) {
    ROS_ERROR("KimeraPgmo: Failed to initialize deformation graph.");
    return false;
  }
  return true;
}

// Initialize publishers
bool KimeraPgmo::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  optimized_mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("optimized_mesh", 1, false);
  optimized_odom_pub_ =
      nl.advertise<nav_msgs::Odometry>("optimized_odom", 1, false);
  pose_graph_pub_ =
      nl.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);
  optimized_path_pub_ =
      nl.advertise<nav_msgs::Path>("optimized_path", 1, false);
  viz_deformation_graph_pub_ =
      nl.advertise<visualization_msgs::Marker>("deformation_graph", 10, false);
  return true;
}

// Initialize callbacks
bool KimeraPgmo::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  full_mesh_sub_ =
      nl.subscribe("full_mesh", 1, &KimeraPgmo::fullMeshCallback, this);

  incremental_mesh_sub_ = nl.subscribe(
      "incremental_mesh", 5, &KimeraPgmo::incrementalMeshCallback, this);

  pose_graph_incremental_sub_ =
      nl.subscribe("pose_graph_incremental",
                   1000,
                   &KimeraPgmo::incrementalPoseGraphCallback,
                   this);

  path_callback_sub_ =
      nl.subscribe("input_path", 2, &KimeraPgmo::optimizedPathCallback, this);

  // Initialize save mesh service
  save_mesh_srv_ =
      nl.advertiseService("save_mesh", &KimeraPgmo::saveMeshCallback, this);

  // Initialize save trajectory service
  save_traj_srv_ = nl.advertiseService(
      "save_trajectory", &KimeraPgmo::saveTrajectoryCallback, this);
  return true;
}

// To publish optimized mesh
bool KimeraPgmo::publishOptimizedMesh() {
  mesh_msgs::TriangleMesh mesh_msg =
      PolygonMeshToTriangleMeshMsg(optimized_mesh_);

  // Create msg
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = last_mesh_stamp_;
  new_msg.header.frame_id = frame_id_;
  new_msg.mesh = mesh_msg;

  optimized_mesh_pub_.publish(new_msg);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmo::publishOptimizedPath(const size_t& robot_id) const {
  std::vector<gtsam::Pose3> gtsam_path =
      deformation_graph_.getOptimizedTrajectory(GetRobotPrefix(robot_id));

  if (gtsam_path.size() == 0) return false;

  if (optimized_path_pub_.getNumSubscribers() > 0) {
    // Create message type
    nav_msgs::Path path;

    // Fill path poses
    path.poses.reserve(gtsam_path.size());
    for (size_t i = 0; i < gtsam_path.size(); i++) {
      gtsam::Pose3 pose = gtsam_path.at(i);
      gtsam::Point3 trans = pose.translation();
      gtsam::Quaternion quat = pose.rotation().toQuaternion();

      geometry_msgs::PoseStamped ps_msg;
      ps_msg.header.frame_id = frame_id_;
      ps_msg.pose.position.x = trans.x();
      ps_msg.pose.position.y = trans.y();
      ps_msg.pose.position.z = trans.z();
      ps_msg.pose.orientation.x = quat.x();
      ps_msg.pose.orientation.y = quat.y();
      ps_msg.pose.orientation.z = quat.z();
      ps_msg.pose.orientation.w = quat.w();

      path.poses.push_back(ps_msg);
    }

    // Publish path message
    path.header.stamp = ros::Time::now();
    path.header.frame_id = frame_id_;
    optimized_path_pub_.publish(path);
  }

  if (optimized_odom_pub_.getNumSubscribers() > 0) {
    // Publish also the optimized odometry
    nav_msgs::Odometry odometry_msg;
    const gtsam::Pose3 last_pose = gtsam_path[gtsam_path.size() - 1];
    const gtsam::Rot3& rotation = last_pose.rotation();
    const gtsam::Quaternion& quaternion = rotation.toQuaternion();

    // Create header.
    odometry_msg.header.stamp = ros::Time::now();
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
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  // if first node initialize
  //// Note that we assume for all node ids that the keys start with 0
  if (msg->nodes.size() > 0 && msg->nodes[0].key == 0 &&
      trajectory_.find(msg->nodes[0].robot_id) == trajectory_.end()) {
    const size_t& robot_id = msg->nodes[0].robot_id;
    const gtsam::Symbol key_symb(GetRobotPrefix(robot_id), 0);
    const gtsam::Pose3& init_pose = RosToGtsam(msg->nodes[0].pose);
    // Initiate first node but do not add prior
    deformation_graph_.addNewNode(key_symb.key(), init_pose, false);
    // Add to trajectory and timestamp map
    trajectory_[robot_id] = {init_pose};
    if (use_msg_time_) {
      timestamps_[robot_id] = {msg->nodes[0].header.stamp};
    } else {
      timestamps_[robot_id] = {ros::Time::now()};
    }
    // Push node to queue to be connected to mesh vertices later
    unconnected_nodes_[robot_id] = std::queue<size_t>();
    unconnected_nodes_[robot_id].push(0);
    ROS_INFO("Initialized first node in pose graph. ");
  }

  try {
    for (pose_graph_tools::PoseGraphEdge pg_edge : msg->edges) {
      // Get edge information
      const gtsam::Pose3& measure = RosToGtsam(pg_edge.pose);
      const Vertex& prev_node = pg_edge.key_from;
      const Vertex& current_node = pg_edge.key_to;
      const size_t& robot_from = pg_edge.robot_from;
      const size_t& robot_to = pg_edge.robot_to;
      const gtsam::Symbol from_key(GetRobotPrefix(robot_from), prev_node);
      const gtsam::Symbol to_key(GetRobotPrefix(robot_to), current_node);

      if (pg_edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
        // odometry edge
        if (robot_from != robot_to) {
          ROS_ERROR(
              "Odometry edge should not connect two nodes from different "
              "robots. ");
          continue;
        }
        // Sanity check key node
        if (trajectory_[robot_from].size() != current_node) {
          ROS_WARN(
              "New current node does not match current trajectory length. %d "
              "vs %d",
              trajectory_[robot_from].size(),
              current_node);
        }
        // Calculate pose of new node
        const gtsam::Pose3& new_pose =
            trajectory_[robot_from][prev_node].compose(measure);
        // Add to trajectory and timestamp maps
        if (trajectory_[robot_from].size() == current_node)
          trajectory_[robot_from].push_back(new_pose);
        if (use_msg_time_) {
          timestamps_[robot_from].push_back(pg_edge.header.stamp);
        } else {
          timestamps_[robot_from].push_back(ros::Time::now());
        }
        // Add new node to queue to be connected to mesh later
        unconnected_nodes_[robot_from].push(current_node);
        // Add to deformation graph
        if (run_mode_ == RunMode::FULL) {
          // Add the pose estimate of new node and between factor (odometry)
          deformation_graph_.addNewBetween(from_key, to_key, measure, new_pose);
        } else if (run_mode_ == RunMode::MESH) {
          // Only add the pose estimate of new node (gtsam Value)
          // Do not add factor
          deformation_graph_.addNewNode(to_key, new_pose, false);
        } else {
          ROS_ERROR("KimeraPgmo: unrecognized run mode. ");
        }
      } else if (pg_edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE &&
                 run_mode_ == RunMode::FULL) {
        // Loop closure edge (only add if we are in full optimization mode )
        // Add to deformation graph
        deformation_graph_.addNewBetween(from_key, to_key, measure);
        ROS_INFO(
            "KimeraPgmo: Loop closure detected between robot %d node %d and "
            "robot %d node %d.",
            robot_from,
            prev_node,
            robot_to,
            current_node);
        num_loop_closures_++;
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR("Error in KimeraPgmo incrementalPoseGraphCallback. ");
    ROS_ERROR(e.what());
  }

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

void KimeraPgmo::optimizedPathCallback(
    const nav_msgs::Path::ConstPtr& path_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  if (trajectory_.size() > 1) {
    ROS_ERROR(
        "KimeraPgmo: Path subscriber does not support centralized multirobot "
        "scenario. ");
  }
  ROS_INFO(
      "KimeraPgmo: Received optimized trajectory. Fixing pose graph nodes...");

  size_t robot_id = trajectory_.begin()->first;
  deformation_graph_.removePriorsWithPrefix(GetRobotPrefix(robot_id));

  std::vector<std::pair<gtsam::Key, gtsam::Pose3> > node_estimates;
  for (size_t i = 0; i < path_msg->poses.size(); i++) {
    if (i == trajectory_[robot_id].size()) break;
    node_estimates.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
        gtsam::Symbol(GetRobotPrefix(robot_id), i).key(),
        RosToGtsam(path_msg->poses[i].pose)));
  }
  deformation_graph_.addNodeMeasurements(node_estimates);

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  path_cb_time_ = spin_duration.count();

  // Log to file
  if (log_output_) {
    std::string log_file = output_prefix_ + std::string("/kimera_pgmo_log.csv");
    logStats(log_file);
  }
}

void KimeraPgmo::fullMeshCallback(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  input_mesh_ = TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
  last_mesh_stamp_ = mesh_msg->header.stamp;
  const size_t& robot_id = mesh_msg->id;

  // Update optimized mesh
  try {
    optimized_mesh_ =
        deformation_graph_.deformMesh(input_mesh_, GetVertexPrefix(robot_id));
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Failed to deform mesh. Out of range error. ");
    optimized_mesh_ = input_mesh_;
  }
  if (optimized_mesh_pub_.getNumSubscribers() > 0) {
    publishOptimizedMesh();
  }

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  full_mesh_cb_time_ = spin_duration.count();

  // Publish deformation graph visualization
  visualizeDeformationGraph();

  return;
}

void KimeraPgmo::incrementalMeshCallback(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  const size_t& robot_id = mesh_msg->id;
  const pcl::PolygonMesh incremental_mesh =
      TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;

  double msg_time;
  if (use_msg_time_) {
    msg_time = mesh_msg->header.stamp.toSec();
  } else {
    msg_time = ros::Time::now().toSec();
  }
  compression_->pruneStoredMesh(msg_time - compression_time_horizon_);
  compression_->compressAndIntegrate(
      incremental_mesh, new_vertices, &new_triangles, &new_indices, msg_time);

  deformation_graph_.updateMesh(
      *new_vertices, new_triangles, GetVertexPrefix(robot_id));
  if (new_indices.size() == 0) {
    // No vertices to connect.
    // Stop timer and save
    auto stop = std::chrono::high_resolution_clock::now();
    auto spin_duration =
        std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
    inc_mesh_cb_time_ = spin_duration.count();
    return;
  }
  bool connection = false;
  // Associate nodes to mesh
  while (!unconnected_nodes_[robot_id].empty()) {
    const size_t node = unconnected_nodes_[robot_id].front();
    unconnected_nodes_[robot_id].pop();
    if (abs(timestamps_[robot_id][node].toSec() - msg_time) < embed_delta_t_) {
      ROS_INFO("Connecting robot %d node %d to %d vertices. ",
               robot_id,
               node,
               new_indices.size());
      deformation_graph_.addNodeValence(
          gtsam::Symbol(GetRobotPrefix(robot_id), node),
          new_indices,
          GetVertexPrefix(robot_id));
      connection = true;
    }
    // termination guarantee
    if (timestamps_[robot_id][node].toSec() > msg_time + embed_delta_t_) break;
  }
  if (!connection) {
    ROS_WARN("KimeraPgmo: Partial mesh not connected to pose graph. ");
  }

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  inc_mesh_cb_time_ = spin_duration.count();

  return;
}

void KimeraPgmo::publishTransforms() {
  for (auto traj : trajectory_) {
    const size_t& robot_id = traj.first;
    const std::vector<gtsam::Pose3>& gtsam_path =
        deformation_graph_.getOptimizedTrajectory(GetRobotPrefix(robot_id));
    const gtsam::Pose3& latest_pose = gtsam_path[traj.second.size() - 1];

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

bool KimeraPgmo::saveMeshCallback(std_srvs::Empty::Request&,
                                  std_srvs::Empty::Response&) {
  // Save mesh
  std::string ply_name = output_prefix_ + std::string("/mesh_pgmo.ply");
  WriteMeshToPly(ply_name, optimized_mesh_);
  ROS_INFO("KimeraPgmo: Saved mesh to file.");
  return true;
}

bool KimeraPgmo::saveTrajectoryCallback(std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&) {
  // Save trajectory
  for (auto traj : trajectory_) {
    const size_t& robot_id = traj.first;
    const std::vector<gtsam::Pose3>& optimized_path =
        deformation_graph_.getOptimizedTrajectory(GetRobotPrefix(robot_id));
    std::ofstream csvfile;
    std::string csv_name = output_prefix_ + std::string("/traj_pgmo.csv");
    csvfile.open(csv_name);
    csvfile << "timestamp[ns],x,y,z,qw,qx,qy,qz\n";
    for (size_t i = 0; i < optimized_path.size(); i++) {
      const gtsam::Point3& pos = optimized_path[i].translation();
      const gtsam::Quaternion& quat =
          optimized_path[i].rotation().toQuaternion();
      const ros::Time& stamp = timestamps_[robot_id][i];
      csvfile << stamp.toNSec() << "," << pos.x() << "," << pos.y() << ","
              << pos.z() << "," << quat.w() << "," << quat.x() << ","
              << quat.y() << "," << quat.z() << "\n";
    }
    csvfile.close();
  }
  ROS_INFO("KimeraPgmo: Saved trajectories to file.");
  return true;
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
  size_t num_keyframes = 0;
  for (auto traj : trajectory_) {
    num_keyframes = num_keyframes + traj.second.size();
  }

  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << trajectory_.size() << "," << num_keyframes << ","
       << num_loop_closures_ << ","
       << deformation_graph_.getGtsamFactors().size() << ","
       << optimized_mesh_.cloud.width * optimized_mesh_.cloud.height << ","
       << deformation_graph_.getVertices().points.size() << ","
       << inc_mesh_cb_time_ << "," << full_mesh_cb_time_ << "," << pg_cb_time_
       << "," << path_cb_time_ << std::endl;
  file.close();

  file.close();
}

void KimeraPgmo::visualizeDeformationGraph() const {
  if (viz_deformation_graph_pub_.getNumSubscribers() > 0) {
    // First get the latest estimates and factors
    const gtsam::Values& graph_values = deformation_graph_.getGtsamValues();
    const gtsam::NonlinearFactorGraph& graph_factors =
        deformation_graph_.getGtsamFactors();

    visualization_msgs::Marker graph_viz;
    graph_viz.header.frame_id = frame_id_;
    graph_viz.header.stamp = ros::Time::now();
    graph_viz.id = 0;
    graph_viz.action = visualization_msgs::Marker::ADD;
    graph_viz.type = visualization_msgs::Marker::LINE_LIST;
    graph_viz.scale.x = 0.02;

    for (auto factor : graph_factors) {
      // Only interested in edges here
      if (factor->keys().size() != 2) continue;

      const gtsam::Symbol& front = factor->front();
      const gtsam::Symbol& back = factor->back();

      const bool front_is_pose_vertex =
          (robot_prefix_to_id.find(front.chr()) != robot_prefix_to_id.end());
      const bool back_is_pose_vertex =
          (robot_prefix_to_id.find(back.chr()) != robot_prefix_to_id.end());

      graph_viz.points.push_back(
          GtsamToRos(graph_values.at<gtsam::Pose3>(front)).position);
      graph_viz.points.push_back(
          GtsamToRos(graph_values.at<gtsam::Pose3>(back)).position);

      // Three types: pose-to-pose, pose-to-mesh, mesh-to-mesh
      // color accordingly
      std_msgs::ColorRGBA color;
      if (front_is_pose_vertex && back_is_pose_vertex) {
        // pose-to-pose
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 0.8;
      } else if (!front_is_pose_vertex && !back_is_pose_vertex) {
        // mesh-to-mesh
        color.r = 0.0;
        color.g = 1.0;
        color.b = 0.0;
        color.a = 0.8;
      } else {
        // pose-to-mesh
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.2;
        color.a = 0.3;
      }
      graph_viz.colors.push_back(color);
      graph_viz.colors.push_back(color);
    }

    viz_deformation_graph_pub_.publish(graph_viz);
  }
}

}  // namespace kimera_pgmo