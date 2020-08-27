/**
 * @file   KimeraPgmo.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

#include "kimera_pgmo/KimeraPgmo.h"

namespace kimera_pgmo {

// Constructor
KimeraPgmo::KimeraPgmo() {}
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

  ROS_INFO("Initializes Mesher Mapper.");

  return true;
}

// Load deformation parameters
bool KimeraPgmo::loadParameters(const ros::NodeHandle& n) {
  if (!n.getParam("frame_id", frame_id_)) return false;
  if (!n.getParam("compression_time_horizon", compression_time_horizon_))
    return false;
  if (n.getParam("output_prefix", output_prefix_)) {
    ROS_INFO("Saving optimized data to: %s .ply and .csv",
             output_prefix_.c_str());
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
  optimized_path_pub_ =
      nl.advertise<nav_msgs::Path>("optimized_path", 1, false);
  optimized_odom_pub_ =
      nl.advertise<nav_msgs::Odometry>("optimized_odom", 1, false);
  pose_graph_pub_ =
      nl.advertise<pose_graph_tools::PoseGraph>("pose_graph", 1, false);
  return true;
}

// Initialize callbacks
bool KimeraPgmo::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  full_mesh_sub_ =
      nl.subscribe("full_mesh", 5, &KimeraPgmo::fullMeshCallback, this);

  incremental_mesh_sub_ = nl.subscribe(
      "incremental_mesh", 5, &KimeraPgmo::incrementalMeshCallback, this);

  pose_graph_incremental_sub_ =
      nl.subscribe("pose_graph_incremental",
                   1000,
                   &KimeraPgmo::incrementalPoseGraphCallback,
                   this);

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
bool KimeraPgmo::publishOptimizedPath() const {
  std::vector<gtsam::Pose3> gtsam_path =
      deformation_graph_.getOptimizedTrajectory();

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
  if (msg->edges.size() == 0) {
    ROS_WARN("No edges in incremental pose graph msg. ");
    return;
  }

  // Check if there are loop closures
  bool loop_closure = (msg->edges.size() > 1);

  // Extract latest odom edge and add to factor graph
  pose_graph_tools::PoseGraphEdge odom_edge = msg->edges[0];
  if (odom_edge.type != pose_graph_tools::PoseGraphEdge::ODOM) {
    ROS_WARN("Expects the odom edge to be first in message.");
    return;
  }
  const gtsam::Pose3 new_odom = RosToGtsam(odom_edge.pose);
  const Vertex prev_node = odom_edge.key_from;
  const Vertex current_node = odom_edge.key_to;
  // Initialize if first node
  if (prev_node == 0 && trajectory_.size() == 0) {
    if (msg->nodes[0].key != 0) {
      ROS_WARN("KimeraPgmo: is the first node not of key 0? ");
    }
    gtsam::Pose3 init_pose = RosToGtsam(msg->nodes[0].pose);
    // Initiate first node but do not add prior
    deformation_graph_.initFirstNode(init_pose, false);
    trajectory_.push_back(init_pose);
    timestamps_.push_back(odom_edge.header.stamp);
    unconnected_nodes_.push(prev_node);
    ROS_INFO("Initialized first node in pose graph. ");
  }

  // Sanity check key node
  if (trajectory_.size() != current_node) {
    // TODO (For now we are assuming that we don't have any prefixes and
    // nodes start from 0 )
    ROS_WARN(
        "New current node does not match current trajectory length. %d vs %d",
        trajectory_.size(),
        current_node);
    return;
  }
  gtsam::Pose3 new_pose = trajectory_[prev_node].compose(new_odom);
  trajectory_.push_back(new_pose);
  timestamps_.push_back(odom_edge.header.stamp);
  unconnected_nodes_.push(current_node);
  deformation_graph_.addNewBetween(prev_node, current_node, new_odom, new_pose);

  if (loop_closure) {
    pose_graph_tools::PoseGraphEdge lc_edge = msg->edges[1];
    const gtsam::Pose3 meas = RosToGtsam(lc_edge.pose);
    const Vertex from_node = lc_edge.key_from;
    const Vertex to_node = lc_edge.key_to;
    deformation_graph_.addNewBetween(from_node, to_node, meas);
    ROS_INFO(
        "KimeraPgmo: Loop closure detected between node %d and %d, optimized "
        "with trajectory of "
        "length %d.",
        from_node,
        to_node,
        trajectory_.size());
  }
}

void KimeraPgmo::fullMeshCallback(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  input_mesh_ = TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
  last_mesh_stamp_ = mesh_msg->header.stamp;

  // Update optimized mesh
  try {
    optimized_mesh_ = deformation_graph_.deformMesh(input_mesh_);
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Failed to deform mesh. Out of range error. ");
    optimized_mesh_ = input_mesh_;
  }
  if (optimized_mesh_pub_.getNumSubscribers() > 0) {
    publishOptimizedMesh();
  }
  if (optimized_path_pub_.getNumSubscribers() > 0 ||
      optimized_odom_pub_.getNumSubscribers() > 0) {
    publishOptimizedPath();
  }

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    // Publish pose graph
    GraphMsgPtr pose_graph_ptr = deformation_graph_.getPoseGraph(timestamps_);
    pose_graph_pub_.publish(*pose_graph_ptr);
  }
  return;
}

void KimeraPgmo::incrementalMeshCallback(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  pcl::PolygonMesh incremental_mesh =
      TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::vector<pcl::Vertices> new_triangles;
  std::vector<size_t> new_indices;

  double msg_time = mesh_msg->header.stamp.toSec();
  compression_->pruneStoredMesh(msg_time - compression_time_horizon_);
  compression_->compressAndIntegrate(
      incremental_mesh, new_vertices, &new_triangles, &new_indices, msg_time);

  deformation_graph_.updateMesh(*new_vertices, new_triangles);
  // Associate nodes to mesh
  while (!unconnected_nodes_.empty()) {
    size_t node = unconnected_nodes_.front();
    unconnected_nodes_.pop();
    if (timestamps_[node].toSec() > msg_time - embed_delta_t_) {
      ROS_INFO("Connection node %d to %d vertices. ", node, new_indices.size());
      deformation_graph_.addNodeValence(node, new_indices);
    }
    // termination guarantee
    if (timestamps_[node].toSec() > msg_time + embed_delta_t_) break;
  }
  return;
}

bool KimeraPgmo::saveMeshCallback(std_srvs::Empty::Request&,
                                  std_srvs::Empty::Response&) {
  // Save mesh
  std::string ply_name = output_prefix_ + std::string(".ply");
  WriteMeshToPly(ply_name, optimized_mesh_);
  ROS_INFO("KimeraPgmo: Saved mesh to file.");
  return true;
}

bool KimeraPgmo::saveTrajectoryCallback(std_srvs::Empty::Request&,
                                        std_srvs::Empty::Response&) {
  // Save trajectory
  std::vector<gtsam::Pose3> optimized_path =
      deformation_graph_.getOptimizedTrajectory();
  std::ofstream csvfile;
  std::string csv_name = output_prefix_ + std::string(".csv");
  csvfile.open(csv_name);
  csvfile << "timestamp[ns],x,y,z,qw,qx,qy,qz\n";
  for (size_t i = 0; i < optimized_path.size(); i++) {
    gtsam::Point3 pos = trajectory_[i].translation();
    gtsam::Quaternion quat = trajectory_[i].rotation().toQuaternion();
    ros::Time stamp = timestamps_[i];
    csvfile << stamp.toNSec() << "," << pos.x() << "," << pos.y() << ","
            << pos.z() << "," << quat.w() << "," << quat.x() << "," << quat.y()
            << "," << quat.z() << "\n";
  }
  csvfile.close();
  ROS_INFO("KimeraPgmo: Saved trajectory to file.");
  return true;
}

}  // namespace kimera_pgmo