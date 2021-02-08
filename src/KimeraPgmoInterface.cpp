/**
 * @file   KimeraPgmoInterface.cpp
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#include <chrono>
#include <cmath>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include "kimera_pgmo/KimeraPgmoInterface.h"

namespace kimera_pgmo {

// Constructor
KimeraPgmoInterface::KimeraPgmoInterface() {}

KimeraPgmoInterface::~KimeraPgmoInterface() {}

// Load deformation parameters
bool KimeraPgmoInterface::loadParameters(const ros::NodeHandle& n) {
  // Set run mode
  int run_mode_num;
  if (!n.getParam("run_mode", run_mode_num)) return false;
  run_mode_ = static_cast<RunMode>(run_mode_num);
  if (!n.getParam("use_msg_time", use_msg_time_)) return false;
  if (!n.getParam("compression_time_horizon", compression_time_horizon_))
    return false;

  if (!n.getParam("embed_trajectory_delta_t", embed_delta_t_)) return false;

  // start the mesh compression module for deformation graph
  if (!n.getParam("d_graph_resolution", deformation_graph_resolution_))
    return false;

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

// To publish optimized mesh
bool KimeraPgmoInterface::publishMesh(const pcl::PolygonMesh& mesh,
                                      const std_msgs::Header& header,
                                      const ros::Publisher* publisher) const {
  if (publisher->getNumSubscribers() == 0) return false;
  if (mesh.cloud.height * mesh.cloud.width == 0) return false;
  mesh_msgs::TriangleMesh mesh_msg = PolygonMeshToTriangleMeshMsg(mesh);

  // Create msg
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header = header;
  new_msg.mesh = mesh_msg;

  publisher->publish(new_msg);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmoInterface::publishPath(const std::vector<gtsam::Pose3>& path,
                                      const std_msgs::Header& header,
                                      const ros::Publisher* publisher) const {
  if (path.size() == 0) return false;

  if (publisher->getNumSubscribers() == 0) return false;
  // Create message type
  nav_msgs::Path path_msg;

  // Fill path poses
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

void KimeraPgmoInterface::processIncrementalPoseGraph(
    const pose_graph_tools::PoseGraph::ConstPtr& msg,
    std::vector<gtsam::Pose3>* initial_trajectory,
    std::queue<size_t>* unconnected_nodes,
    std::vector<ros::Time>* node_timestamps) {
  // if first node initialize
  //// Note that we assume for all node ids that the keys start with 0
  if (msg->nodes.size() > 0 && msg->nodes[0].key == 0 &&
      initial_trajectory->size() == 0) {
    const size_t& robot_id = msg->nodes[0].robot_id;
    const gtsam::Symbol key_symb(GetRobotPrefix(robot_id), 0);
    const gtsam::Pose3& init_pose = RosToGtsam(msg->nodes[0].pose);
    // Initiate first node but do not add prior
    deformation_graph_.addNewNode(key_symb.key(), init_pose, false);
    // Add to trajectory and timestamp map
    initial_trajectory->push_back(init_pose);
    if (use_msg_time_) {
      node_timestamps->push_back(msg->nodes[0].header.stamp);
    } else {
      node_timestamps->push_back(ros::Time::now());
    }
    // Push node to queue to be connected to mesh vertices later
    unconnected_nodes->push(0);
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
        if (initial_trajectory->size() != current_node) {
          ROS_WARN(
              "New current node does not match current trajectory length. %d "
              "vs %d",
              initial_trajectory->size(),
              current_node);
        }
        // Calculate pose of new node
        const gtsam::Pose3& new_pose =
            initial_trajectory->at(prev_node).compose(measure);
        // Add to trajectory and timestamp maps
        if (initial_trajectory->size() == current_node)
          initial_trajectory->push_back(new_pose);
        if (use_msg_time_) {
          node_timestamps->push_back(pg_edge.header.stamp);
        } else {
          node_timestamps->push_back(ros::Time::now());
        }
        // Add new node to queue to be connected to mesh later
        unconnected_nodes->push(current_node);
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
}

void KimeraPgmoInterface::processOptimizedPath(
    const nav_msgs::Path::ConstPtr& path_msg,
    const size_t& robot_id) {
  ROS_INFO(
      "KimeraPgmo: Received optimized trajectory. Fixing pose graph nodes...");

  deformation_graph_.removePriorsWithPrefix(GetRobotPrefix(robot_id));

  std::vector<std::pair<gtsam::Key, gtsam::Pose3> > node_estimates;
  for (size_t i = 0; i < path_msg->poses.size(); i++) {
    node_estimates.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
        gtsam::Symbol(GetRobotPrefix(robot_id), i).key(),
        RosToGtsam(path_msg->poses[i].pose)));
  }
  deformation_graph_.addNodeMeasurements(node_estimates);
}

pcl::PolygonMesh KimeraPgmoInterface::optimizeAndPublishFullMesh(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg,
    const ros::Publisher* publisher) {
  const pcl::PolygonMesh& input_mesh =
      TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
  const size_t& robot_id = mesh_msg->id;
  std_msgs::Header mesh_header = mesh_msg->header;

  // Optimize mesh
  pcl::PolygonMesh optimized_mesh;
  try {
    optimized_mesh =
        deformation_graph_.deformMesh(input_mesh, GetVertexPrefix(robot_id));
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Failed to deform mesh. Out of range error. ");
  }
  if (publisher->getNumSubscribers() > 0) {
    publishMesh(optimized_mesh, mesh_header, publisher);
  }
  return optimized_mesh;
}

void KimeraPgmoInterface::processIncrementalMesh(
    const kimera_pgmo::TriangleMeshIdStamped::ConstPtr& mesh_msg,
    const OctreeCompressionPtr compressor,
    const std::vector<ros::Time>& node_timestamps,
    std::queue<size_t>* unconnected_nodes) {
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
  compressor->pruneStoredMesh(msg_time - compression_time_horizon_);
  compressor->compressAndIntegrate(
      incremental_mesh, new_vertices, &new_triangles, &new_indices, msg_time);
  deformation_graph_.updateMesh(
      *new_vertices, new_triangles, GetVertexPrefix(robot_id));
  if (new_indices.size() == 0) return;
  bool connection = false;
  // Associate nodes to mesh
  while (!unconnected_nodes->empty()) {
    const size_t node = unconnected_nodes->front();
    unconnected_nodes->pop();
    if (abs(node_timestamps[node].toSec() - msg_time) < embed_delta_t_) {
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
    if (node_timestamps[node].toSec() > msg_time + embed_delta_t_) break;
  }
  if (!connection) {
    ROS_WARN("KimeraPgmo: Partial mesh not connected to pose graph. ");
  }

  return;
}

bool KimeraPgmoInterface::saveMesh(const pcl::PolygonMesh& mesh,
                                   const std::string& ply_name) {
  // Save mesh
  WriteMeshToPly(ply_name, mesh);
  ROS_INFO("KimeraPgmo: Saved mesh to file.");
  return true;
}

bool KimeraPgmoInterface::saveTrajectory(
    const std::vector<gtsam::Pose3>& trajectory,
    const std::vector<ros::Time>& timestamps,
    const std::string& csv_file) {
  // There should be a timestamp associated with each pose
  assert(trajectory.size() == timestamps.size());

  std::ofstream csvfile;
  csvfile.open(csv_file);
  csvfile << "timestamp[ns],x,y,z,qw,qx,qy,qz\n";
  for (size_t i = 0; i < trajectory.size(); i++) {
    const gtsam::Point3& pos = trajectory[i].translation();
    const gtsam::Quaternion& quat = trajectory[i].rotation().toQuaternion();
    const ros::Time& stamp = timestamps[i];
    csvfile << stamp.toNSec() << "," << pos.x() << "," << pos.y() << ","
            << pos.z() << "," << quat.w() << "," << quat.x() << "," << quat.y()
            << "," << quat.z() << "\n";
  }
  csvfile.close();
  ROS_INFO("KimeraPgmo: Saved trajectories to file.");
  return true;
}

void KimeraPgmoInterface::visualizeDeformationGraph(
    const ros::Publisher* publisher) const {
  if (publisher->getNumSubscribers() > 0) {
    // First get the latest estimates and factors
    const gtsam::Values& graph_values = deformation_graph_.getGtsamValues();
    const gtsam::NonlinearFactorGraph& graph_factors =
        deformation_graph_.getGtsamFactors();

    visualization_msgs::Marker graph_viz;
    graph_viz.header.frame_id = "world";
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

    publisher->publish(graph_viz);
  }
}

}  // namespace kimera_pgmo