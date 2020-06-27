/**
 * @file   KimeraRmpgo.cpp
 * @brief  KimeraRmpgo class: Main class and ROS interface
 * @author Yun Chang
 */
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

#include "mesher_mapper/KimeraRmpgo.h"

namespace mesher_mapper {

// Constructor
KimeraRmpgo::KimeraRmpgo() : save_optimized_mesh_(false) {}
KimeraRmpgo::~KimeraRmpgo() {}

// Initialize parameters, publishers, and subscribers and deformation graph
bool KimeraRmpgo::Initialize(const ros::NodeHandle& n) {
  if (!LoadParameters(n)) {
    ROS_ERROR("KimeraRmpgo: Failed to load parameters.");
  }

  if (!CreatePublishers(n)) {
    ROS_ERROR("KimeraRmpgo: Failed to create publishers.");
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("KimeraRmpgo: Failed to register callbacks.");
  }

  ROS_INFO("Initializes Mesher Mapper.");

  return true;
}

// Load deformation parameters
bool KimeraRmpgo::LoadParameters(const ros::NodeHandle& n) {
  if (!n.getParam("frame_id", frame_id_)) return false;
  if (!n.getParam("embed_trajectory/max_delta_t", embed_delta_t_)) return false;
  if (!n.getParam("embed_trajectory/max_delta_r", embed_delta_r_)) return false;
  if (n.getParam("output_ply_file", output_file_)) {
    save_optimized_mesh_ = true;
    ROS_INFO("Saving optimized mesh to: %s", output_file_.c_str());
  }

  // start the mesh compression module for deformation graph
  double deformation_graph_resolution;
  if (!n.getParam("d_graph_resolution", deformation_graph_resolution))
    return false;

  if (!d_graph_compression_.Initialize(
          n, deformation_graph_resolution, "deformation")) {
    ROS_ERROR(
        "KimeraRmpgo: Failed to intialize deformation graph compression "
        "module.");
    return false;
  }

  // start deformation graph module
  double pgo_trans_threshold, pgo_rot_threshold;
  if (!n.getParam("rpgo/translation_threshold", pgo_trans_threshold))
    return false;
  if (!n.getParam("rpgo/rotation_threshold", pgo_rot_threshold)) return false;

  if (!deformation_graph_.Initialize(pgo_trans_threshold, pgo_rot_threshold)) {
    ROS_ERROR("KimeraRmpgo: Failed to initialze deformation graph.");
    return false;
  }
  return true;
}

// Initialize publishers
bool KimeraRmpgo::CreatePublishers(const ros::NodeHandle& n) {
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
bool KimeraRmpgo::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);

  input_mesh_sub_ =
      nl.subscribe("input_mesh", 50, &KimeraRmpgo::MeshCallback, this);

  pose_graph_incremental_sub_ =
      nl.subscribe("pose_graph_incremental",
                   50,
                   &KimeraRmpgo::IncrementalPoseGraphCallback,
                   this);

  // start timer
  update_timer_ = nl.createTimer(1.0, &KimeraRmpgo::ProcessTimerCallback, this);
  return true;
}

// To publish optimized mesh
bool KimeraRmpgo::PublishOptimizedMesh() {
  mesh_msgs::TriangleMesh mesh_msg =
      PolygonMeshToTriangleMeshMsg(optimized_mesh_);

  // Create msg
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = ros::Time::now();
  new_msg.header.frame_id = frame_id_;
  new_msg.mesh = mesh_msg;

  optimized_mesh_pub_.publish(new_msg);
  return true;
}

// To publish optimized trajectory
bool KimeraRmpgo::PublishOptimizedPath() const {
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

void KimeraRmpgo::IncrementalPoseGraphCallback(
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
      ROS_WARN("KimeraRmpgo: is the first node not of key 0? ");
    }
    gtsam::Pose3 init_pose = RosToGtsam(msg->nodes[0].pose);
    deformation_graph_.initFirstNode(init_pose);
    trajectory_.push_back(init_pose);
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
  timestamps_.push_back(msg->header.stamp);
  deformation_graph_.addNewBetween(prev_node, current_node, new_odom, new_pose);

  // Associate the node to the simplified mesh and add to trajectory
  // Get the latest observation timestamp of the vertices
  double msg_time = msg->header.stamp.toSec();
  std::vector<double> latest_observed_times;
  d_graph_compression_.getVerticesTimestamps(&latest_observed_times);
  // Get the vertices of the simplified mesh
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr simplified_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  d_graph_compression_.getVertices(simplified_vertices);
  // TODO might not be best but for now need to do this
  // since meshes is running slow, fix later to synch properly
  for (size_t i = 0; i < trajectory_.size(); i++) {
    Vertices valences;
    gtsam::Point3 pos = trajectory_[i].translation();
    // connect to nodes that are proximate in spacetime
    for (size_t j = 0; j < latest_observed_times.size(); j++) {
      if (abs(timestamps_[i].toSec() - latest_observed_times[j]) <
          embed_delta_t_) {
        pcl::PointXYZRGBA candidate_pos = simplified_vertices->points[j];
        double dist = std::sqrt(
            (pos.x() - candidate_pos.x) * (pos.x() - candidate_pos.x) +
            (pos.y() - candidate_pos.y) * (pos.y() - candidate_pos.y) +
            (pos.z() - candidate_pos.z) * (pos.z() - candidate_pos.z));
        if (dist < embed_delta_r_) {
          valences.push_back(j);
        }
      }
    }
    ROS_DEBUG_STREAM("Trajectory node " << i << " connected to "
                                        << valences.size() << " points out of "
                                        << latest_observed_times.size()
                                        << " in simplified mesh");
    deformation_graph_.updateNodeValence(i, valences, true);
  }

  pcl::PolygonMesh simplified_mesh;
  // Fill in simplified mesh
  std::vector<pcl::Vertices> simplified_polygons;
  d_graph_compression_.getPolygons(&simplified_polygons);
  simplified_mesh.polygons = simplified_polygons;
  pcl::toPCLPointCloud2(*simplified_vertices, simplified_mesh.cloud);

  // Check if new portions added for deformation graph
  deformation_graph_.updateMesh(simplified_mesh);
  if (loop_closure) {
    pose_graph_tools::PoseGraphEdge lc_edge = msg->edges[1];
    const gtsam::Pose3 meas = RosToGtsam(odom_edge.pose);
    const Vertex from_node = lc_edge.key_from;
    const Vertex to_node = lc_edge.key_to;
    deformation_graph_.addNewBetween(from_node, to_node, meas);
    deformation_graph_.update();
    ROS_INFO(
        "KimeraRmpgo: Loop closure detected between node %d and %d, optimized "
        "with trajectory of "
        "length %d.",
        from_node,
        to_node,
        trajectory_.size());
  } else {
    deformation_graph_.update();
  }
}

void KimeraRmpgo::MeshCallback(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  input_mesh_ = TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
  return;
}

void KimeraRmpgo::ProcessTimerCallback(const ros::TimerEvent& ev) {
  // Update optimized mesh
  optimized_mesh_ = deformation_graph_.deformMesh(input_mesh_);
  if (optimized_mesh_pub_.getNumSubscribers() > 0) {
    PublishOptimizedMesh();
  }
  if (optimized_path_pub_.getNumSubscribers() > 0 ||
      optimized_odom_pub_.getNumSubscribers() > 0) {
    PublishOptimizedPath();
  }

  if (pose_graph_pub_.getNumSubscribers() > 0) {
    // Publish pose graph
    GraphMsgPtr pose_graph_ptr = deformation_graph_.getPoseGraph();
    pose_graph_pub_.publish(*pose_graph_ptr);
  }

  // Save mesh
  if (save_optimized_mesh_) {
    WriteMeshToPly(output_file_, optimized_mesh_);
  }
}

}  // namespace mesher_mapper