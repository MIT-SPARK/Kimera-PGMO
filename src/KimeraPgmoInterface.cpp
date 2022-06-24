/**
 * @file   KimeraPgmoInterface.cpp
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#include <chrono>
#include <cmath>
#include <limits>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <visualization_msgs/Marker.h>

#include "kimera_pgmo/KimeraPgmoInterface.h"

namespace kimera_pgmo {

// Constructor
KimeraPgmoInterface::KimeraPgmoInterface()
    : deformation_graph_(new DeformationGraph),
      full_mesh_updated_(false),
      num_loop_closures_(0) {}

KimeraPgmoInterface::~KimeraPgmoInterface() {}

// Load deformation parameters
bool KimeraPgmoInterface::loadParameters(const ros::NodeHandle& n) {
  // Set run mode
  int run_mode_num;
  if (!n.getParam("run_mode", run_mode_num)) return false;
  run_mode_ = static_cast<RunMode>(run_mode_num);
  if (!n.getParam("use_msg_time", use_msg_time_)) return false;

  if (!n.getParam("embed_trajectory_delta_t", embed_delta_t_)) return false;

  if (!n.getParam("num_interp_pts", num_interp_pts_)) return false;
  assert(num_interp_pts_ > 1);

  if (!n.getParam("interp_horizon", interp_horizon_)) return false;

  if (!n.getParam("add_initial_prior", b_add_initial_prior_)) return false;

  if (!n.getParam("covariance/odom", odom_variance_)) return false;
  if (!n.getParam("covariance/loop_close", lc_variance_)) return false;
  if (!n.getParam("covariance/prior", prior_variance_)) return false;
  if (!n.getParam("covariance/mesh_mesh", mesh_edge_variance_)) return false;
  if (!n.getParam("covariance/pose_mesh", pose_mesh_variance_)) return false;

  // start deformation graph module
  double odom_trans_threshold, odom_rot_threshold, pcm_trans_threshold,
      pcm_rot_threshold, gnc_alpha;
  if (!n.getParam("rpgo/odom_trans_threshold", odom_trans_threshold))
    return false;
  if (!n.getParam("rpgo/odom_rot_threshold", odom_rot_threshold)) return false;
  if (!n.getParam("rpgo/pcm_trans_threshold", pcm_trans_threshold))
    return false;
  if (!n.getParam("rpgo/pcm_rot_threshold", pcm_rot_threshold)) return false;
  if (!n.getParam("rpgo/gnc_alpha", gnc_alpha)) return false;

  //  Addditional optional gnc values
  int gnc_max_it;
  double gnc_mu_step, gnc_cost_tol, gnc_weight_tol;
  bool lm_diagonal_damping, gnc_fix_prev_inliers;
  n.param<bool>("rpgo/lm_diagonal_damping", lm_diagonal_damping, true);
  n.param<int>("rpgo/gnc_max_iterations", gnc_max_it, 100);
  n.param<double>("rpgo/gnc_mu_step", gnc_mu_step, 1.4);
  n.param<double>("rpgo/gnc_cost_tolerance", gnc_cost_tol, 1.0e-5);
  n.param<double>("rpgo/gnc_weight_tolerance", gnc_weight_tol, 1.0e-4);
  n.param<bool>("rpgo/gnc_fix_prev_inliers", gnc_fix_prev_inliers, false);

  std::string log_path;
  n.param<std::string>("output_prefix", log_path, "");

  // Initialize pgo_:
  KimeraRPGO::RobustSolverParams pgo_params;
  pgo_params.setPcmSimple3DParams(odom_trans_threshold,
                                  odom_rot_threshold,
                                  pcm_trans_threshold,
                                  pcm_rot_threshold,
                                  KimeraRPGO::Verbosity::UPDATE);
  pgo_params.setLmDiagonalDamping(lm_diagonal_damping);
  // Use GNC (confidence value)
  if (gnc_alpha > 0 && gnc_alpha < 1) {
    pgo_params.setGncInlierCostThresholdsAtProbability(
        gnc_alpha,
        static_cast<size_t>(gnc_max_it),
        gnc_mu_step,
        gnc_cost_tol,
        gnc_weight_tol,
        gnc_fix_prev_inliers);
  }

  // Log output
  if (!log_path.empty()) {
    pgo_params.logOutput(log_path);
  }

  if (!deformation_graph_->initialize(pgo_params)) {
    ROS_ERROR("KimeraPgmo: Failed to initialize deformation graph.");
    return false;
  }

  // If inliers are not fixed, need to perform interpolation on whole mesh
  // everytime we optimize
  deformation_graph_->setForceRecalculate(!gnc_fix_prev_inliers);

  // We only optimize when we explicitly call deformation_graph_->optimize()
  deformation_graph_->storeOnlyNoOptimization();
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
  ROS_INFO("Publish optimized mesh. ");
  publisher->publish(new_msg);
  return true;
}

// To publish optimized trajectory
bool KimeraPgmoInterface::publishPath(const Path& path,
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
    Path* initial_trajectory,
    std::queue<size_t>* unconnected_nodes,
    std::vector<ros::Time>* node_timestamps) {
  // if first node initialize
  //// Note that we assume for all node ids that the keys start with 0
  if (msg->nodes.size() > 0 && msg->nodes[0].key == 0 &&
      initial_trajectory->size() == 0) {
    size_t robot_id = msg->nodes[0].robot_id;

    const gtsam::Symbol key_symb(GetRobotPrefix(robot_id), 0);
    const gtsam::Pose3& init_pose = RosToGtsam(msg->nodes[0].pose);
    // Initiate first node but do not add prior
    deformation_graph_->addNewNode(
        key_symb.key(), init_pose, b_add_initial_prior_, prior_variance_);
    // Add to trajectory and timestamp map
    initial_trajectory->push_back(init_pose);
    if (use_msg_time_) {
      node_timestamps->push_back(msg->nodes[0].header.stamp);
    } else {
      node_timestamps->push_back(ros::Time::now());
    }
    // Push node to queue to be connected to mesh vertices later
    unconnected_nodes->push(0);
    ROS_DEBUG("Initialized first node in pose graph. ");
  }

  try {
    for (pose_graph_tools::PoseGraphEdge pg_edge : msg->edges) {
      // Get edge information
      const gtsam::Pose3& measure = RosToGtsam(pg_edge.pose);
      const Vertex& prev_node = pg_edge.key_from;
      const Vertex& current_node = pg_edge.key_to;

      size_t robot_from = pg_edge.robot_from;
      size_t robot_to = pg_edge.robot_to;

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
              static_cast<int>(initial_trajectory->size()),
              static_cast<int>(current_node));
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
        // Add the pose estimate of new node and between factor (odometry) to
        // deformation graph
        deformation_graph_->addNewBetween(
            from_key, to_key, measure, new_pose, odom_variance_);
      } else if (pg_edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE &&
                 run_mode_ == RunMode::FULL) {
        // Loop closure edge (only add if we are in full optimization mode )
        // Add to deformation graph
        deformation_graph_->addNewBetween(
            from_key, to_key, measure, gtsam::Pose3(), lc_variance_);
        ROS_INFO(
            "KimeraPgmo: Loop closure detected between robot %d node %d and "
            "robot %d node %d.",
            static_cast<int>(robot_from),
            static_cast<int>(prev_node),
            static_cast<int>(robot_to),
            static_cast<int>(current_node));
        num_loop_closures_++;
      }
    }
  } catch (const std::exception& e) {
    ROS_ERROR_STREAM(
        "Error in KimeraPgmo incrementalPoseGraphCallback: " << e.what());
  }
}

void KimeraPgmoInterface::processOptimizedPath(
    const nav_msgs::Path::ConstPtr& path_msg,
    const size_t& robot_id) {
  ROS_INFO(
      "KimeraPgmo: Received optimized trajectory. Fixing pose graph nodes...");

  deformation_graph_->removePriorsWithPrefix(GetRobotPrefix(robot_id));

  std::vector<std::pair<gtsam::Key, gtsam::Pose3> > node_estimates;
  for (size_t i = 0; i < path_msg->poses.size(); i++) {
    node_estimates.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
        gtsam::Symbol(GetRobotPrefix(robot_id), i).key(),
        RosToGtsam(path_msg->poses[i].pose)));
  }
  deformation_graph_->addNodeMeasurements(node_estimates, prior_variance_);
}

bool KimeraPgmoInterface::optimizeFullMesh(const KimeraPgmoMesh& mesh_msg,
                                           pcl::PolygonMesh::Ptr optimized_mesh,
                                           std::vector<ros::Time>* mesh_vertex_stamps,
                                           bool do_optimize) {
  std::vector<int> mesh_vertex_graph_inds;
  const pcl::PolygonMesh& input_mesh = PgmoMeshMsgToPolygonMesh(
      mesh_msg, mesh_vertex_stamps, &mesh_vertex_graph_inds);
  // check if empty
  if (input_mesh.cloud.height * input_mesh.cloud.width == 0) return false;

  size_t robot_id = mesh_msg.id;

  std_msgs::Header mesh_header = mesh_msg.header;

  // Optimize mesh
  try {
    if (run_mode_ == RunMode::DPGMO) {
      // Here we are getting the optimized values from the dpgo solver
      *optimized_mesh =
          deformation_graph_->deformMesh(input_mesh,
                                         *mesh_vertex_stamps,
                                         mesh_vertex_graph_inds,
                                         GetVertexPrefix(robot_id),
                                         dpgmo_values_,
                                         num_interp_pts_,
                                         interp_horizon_);
    } else {
      if (do_optimize) {
        deformation_graph_->optimize();
      }
      *optimized_mesh =
          deformation_graph_->deformMesh(input_mesh,
                                         *mesh_vertex_stamps,
                                         mesh_vertex_graph_inds,
                                         GetVertexPrefix(robot_id),
                                         num_interp_pts_,
                                         interp_horizon_);
    }
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Failed to deform mesh. Out of range error. ");
    return false;
  }

  full_mesh_updated_ = true;
  return true;
}

void KimeraPgmoInterface::processIncrementalMeshGraph(
    const pose_graph_tools::PoseGraph::ConstPtr& mesh_graph_msg,
    const std::vector<ros::Time>& node_timestamps,
    std::queue<size_t>* unconnected_nodes) {
  if (mesh_graph_msg->edges.size() == 0 || mesh_graph_msg->nodes.size() == 0) {
    ROS_DEBUG(
        "processIncrementalMeshGraph: 0 nodes or 0 edges in mesh graph msg. ");
    return;
  }
  // Assume graph only contains message from one robot
  size_t robot_id = mesh_graph_msg->nodes[0].robot_id;
  // Mesh edges and nodes
  std::vector<std::pair<gtsam::Key, gtsam::Key> > new_mesh_edges;
  gtsam::Values new_mesh_nodes;
  std::vector<size_t>
      new_indices;  // TODO: this can be cleaned up by changing the defomration
                    // graph addNodeValence interface

  // Convert and add edges
  for (auto e : mesh_graph_msg->edges) {
    if (e.robot_from != robot_id || e.robot_to != robot_id) {
      ROS_WARN(
          "processIncrementalMeshGraph: detect different robot ids in single "
          "mesh graph msg. ");
    }
    gtsam::Key from = gtsam::Symbol(GetVertexPrefix(e.robot_from), e.key_from);
    gtsam::Key to = gtsam::Symbol(GetVertexPrefix(e.robot_to), e.key_to);
    new_mesh_edges.push_back(std::pair<gtsam::Key, gtsam::Key>(from, to));
  }

  // Convert and add nodes
  for (auto n : mesh_graph_msg->nodes) {
    if (n.robot_id != robot_id) {
      ROS_WARN(
          "processIncrementalMeshGraph: detect different robot ids in single "
          "mesh graph msg. ");
    }
    gtsam::Key key = gtsam::Symbol(GetVertexPrefix(n.robot_id), n.key);
    gtsam::Pose3 node_pose = RosToGtsam(n.pose);
    try {
      new_mesh_nodes.insert(key, node_pose);
    } catch (const gtsam::ValuesKeyAlreadyExists& e) {
      ROS_WARN("processing mesh node duplicate\n");
    }
  }

  // Add to deformation graph
  deformation_graph_->addNewMeshEdgesAndNodes(new_mesh_edges,
                                              new_mesh_nodes,
                                              mesh_graph_msg->header.stamp,
                                              &new_indices,
                                              mesh_edge_variance_,
                                              false);

  double msg_time;
  if (use_msg_time_) {
    msg_time = mesh_graph_msg->header.stamp.toSec();
  } else {
    msg_time = ros::Time::now().toSec();
  }

  bool connection = false;
  // Associate nodes to mesh
  if (!unconnected_nodes->empty() && new_indices.size() > 0) {
    // find the closest
    size_t closest_node = unconnected_nodes->front();
    double min_difference = std::numeric_limits<double>::infinity();
    while (!unconnected_nodes->empty()) {
      const size_t node = unconnected_nodes->front();
      if (abs(node_timestamps[node].toSec() - msg_time) < min_difference) {
        min_difference = abs(node_timestamps[node].toSec());
        closest_node = node;
        unconnected_nodes->pop();
      } else {
        break;
      }
    }

    if (verbose_) {
      ROS_INFO("Connecting robot %d node %d to %d vertices. ",
               static_cast<int>(robot_id),
               static_cast<int>(closest_node),
               static_cast<int>(new_indices.size()));
    }
    deformation_graph_->addNodeValence(
        gtsam::Symbol(GetRobotPrefix(robot_id), closest_node),
        new_indices,
        GetVertexPrefix(robot_id),
        pose_mesh_variance_,
        false);
    connection = true;
    if (abs(node_timestamps[closest_node].toSec() - msg_time) >
        embed_delta_t_) {
      ROS_WARN(
          "Connection from robot node to vertices have a time difference "
          "of %f",
          abs(node_timestamps[closest_node].toSec() - msg_time));
    }
  }
  if (!connection && new_indices.size() > 0) {
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
    const Path& trajectory,
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

bool KimeraPgmoInterface::saveDeformationGraph(const std::string& dgrf_name) {
  // Save mesh
  deformation_graph_->save(dgrf_name);
  ROS_INFO("KimeraPgmo: Saved deformation graph to file.");
  return true;
}

bool KimeraPgmoInterface::getConsistencyFactors(
    const size_t& robot_id,
    pose_graph_tools::PoseGraph* pg_mesh_msg,
    const size_t& vertex_index_offset) const {
  assert(nullptr != pg_mesh_msg);
  pg_mesh_msg->edges.clear();
  pg_mesh_msg->nodes.clear();
  // Make sure that robot id is valid
  if (robot_id_to_prefix.find(robot_id) == robot_id_to_prefix.end()) {
    ROS_ERROR("Unexpected robot id. ");
    return false;
  }

  // Get the edges from the deformation graph
  gtsam::NonlinearFactorGraph edge_factors =
      deformation_graph_->getConsistencyFactors();

  // Get the prefixes
  char vertex_prefix = robot_id_to_vertex_prefix.at(robot_id);
  char robot_prefix = robot_id_to_prefix.at(robot_id);

  // Iterate and convert the edges to PoseGraphEdge type
  for (auto factor : edge_factors) {
    // Create edge
    pose_graph_tools::PoseGraphEdge pg_edge;

    gtsam::Symbol from(factor->front());
    gtsam::Symbol to(factor->back());

    if (from.chr() == vertex_prefix) {
      if (to.chr() == vertex_prefix) {
        pg_edge.type = pose_graph_tools::PoseGraphEdge::MESH;
      } else if (to.chr() == robot_prefix) {
        pg_edge.type = pose_graph_tools::PoseGraphEdge::MESH_POSE;
      } else {
        ROS_WARN("Unexpected edge type. ");
        continue;
      }
    } else if (from.chr() == robot_prefix) {
      if (to.chr() == vertex_prefix) {
        pg_edge.type = pose_graph_tools::PoseGraphEdge::POSE_MESH;
      } else if (to.chr() == robot_prefix) {
        ROS_ERROR(
            "Getting a pose-to-pose edge in deformation graph consistency "
            "factors. Check for bug. ");
        continue;
      } else {
        ROS_WARN("Unexpected edge type. ");
        continue;
      }
    }

    pg_edge.robot_from = robot_id;
    pg_edge.robot_to = robot_id;
    pg_edge.key_from = from.index();
    pg_edge.key_to = to.index();
    // Covariance is infinite for rotation part
    pg_edge.covariance[21] = 1.0 / 0.0;
    pg_edge.covariance[28] = 1.0 / 0.0;
    pg_edge.covariance[35] = 1.0 / 0.0;
    // Pose should be [I , R_1^{-1} (t2 - t1)] *** these are all initial
    // poses/positions
    switch (pg_edge.type) {
      case pose_graph_tools::PoseGraphEdge::MESH: {
        const gtsam::Point3& vertex_pos_from =
            deformation_graph_->getInitialPositionVertex(vertex_prefix,
                                                         pg_edge.key_from);
        const gtsam::Point3& vertex_pos_to =
            deformation_graph_->getInitialPositionVertex(vertex_prefix,
                                                         pg_edge.key_to);
        pg_edge.pose = GtsamToRos(
            gtsam::Pose3(gtsam::Rot3(), vertex_pos_to - vertex_pos_from));

        // Update key with offset
        pg_edge.key_from = pg_edge.key_from + vertex_index_offset;
        pg_edge.key_to = pg_edge.key_to + vertex_index_offset;
        pg_mesh_msg->edges.push_back(pg_edge);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::POSE_MESH: {
        const gtsam::Pose3& pose_from =
            deformation_graph_->getInitialPose(robot_prefix, pg_edge.key_from);
        const gtsam::Point3& vertex_pos_to =
            deformation_graph_->getInitialPositionVertex(vertex_prefix,
                                                         pg_edge.key_to);
        pg_edge.pose = GtsamToRos(gtsam::Pose3(
            gtsam::Rot3(),
            pose_from.rotation().inverse().rotate(vertex_pos_to -
                                                  pose_from.translation())));

        // Update key with offset
        pg_edge.key_to = pg_edge.key_to + vertex_index_offset;
        pg_mesh_msg->edges.push_back(pg_edge);
        break;
      }
      case pose_graph_tools::PoseGraphEdge::MESH_POSE: {
        const gtsam::Point3& vertex_pos_from =
            deformation_graph_->getInitialPositionVertex(vertex_prefix,
                                                         pg_edge.key_from);
        const gtsam::Pose3& pose_to =
            deformation_graph_->getInitialPose(robot_prefix, pg_edge.key_to);
        pg_edge.pose = GtsamToRos(gtsam::Pose3(
            gtsam::Rot3(), pose_to.translation() - vertex_pos_from));

        // Update key with offset
        pg_edge.key_from = pg_edge.key_from + vertex_index_offset;
        pg_mesh_msg->edges.push_back(pg_edge);
        break;
      }
    }
  }
  if (pg_mesh_msg->edges.size() == 0) return false;

  // Get the nodes from the deformation graph
  const std::vector<gtsam::Point3>& initial_positions =
      deformation_graph_->getInitialPositionsVertices(vertex_prefix);

  for (size_t i = 0; i < initial_positions.size(); i++) {
    pose_graph_tools::PoseGraphNode pg_node;
    pg_node.robot_id = robot_id;
    pg_node.key = i + vertex_index_offset;
    pg_node.pose =
        GtsamToRos(gtsam::Pose3(gtsam::Rot3(), initial_positions[i]));
    pg_mesh_msg->nodes.push_back(pg_node);
  }
  return true;
}

void KimeraPgmoInterface::visualizeDeformationGraphMeshEdges(
    const ros::Publisher* mesh_mesh_pub,
    const ros::Publisher* pose_mesh_pub) const {
  if (mesh_mesh_pub->getNumSubscribers() > 0 ||
      pose_mesh_pub->getNumSubscribers() > 0) {
    // First get the latest estimates and factors
    const gtsam::Values& graph_values = deformation_graph_->getGtsamValues();
    const gtsam::NonlinearFactorGraph& graph_factors =
        deformation_graph_->getGtsamFactors();

    // Msg for the mesh to mesh edges
    visualization_msgs::Marker mesh_mesh_viz;
    mesh_mesh_viz.header.frame_id = "world";
    mesh_mesh_viz.header.stamp = ros::Time::now();
    mesh_mesh_viz.id = 0;
    mesh_mesh_viz.action = visualization_msgs::Marker::ADD;
    mesh_mesh_viz.type = visualization_msgs::Marker::LINE_LIST;
    mesh_mesh_viz.scale.x = 0.02;
    // Msg for the pose to mesh edges
    visualization_msgs::Marker pose_mesh_viz;
    pose_mesh_viz.header.frame_id = "world";
    pose_mesh_viz.header.stamp = ros::Time::now();
    pose_mesh_viz.id = 0;
    pose_mesh_viz.action = visualization_msgs::Marker::ADD;
    pose_mesh_viz.type = visualization_msgs::Marker::LINE_LIST;
    pose_mesh_viz.scale.x = 0.02;

    for (auto factor : graph_factors) {
      // Only interested in edges here
      if (factor->keys().size() != 2) continue;

      const gtsam::Symbol& front = factor->front();
      const gtsam::Symbol& back = factor->back();

      const bool front_is_pose_vertex =
          (robot_prefix_to_id.find(front.chr()) != robot_prefix_to_id.end());
      const bool back_is_pose_vertex =
          (robot_prefix_to_id.find(back.chr()) != robot_prefix_to_id.end());

      // Three types: pose-to-pose, pose-to-mesh, mesh-to-mesh
      // color accordingly
      std_msgs::ColorRGBA color;
      if (front_is_pose_vertex && back_is_pose_vertex) {
        // pose-to-pose
        continue;
      } else if (!front_is_pose_vertex && !back_is_pose_vertex) {
        // mesh-to-mesh
        mesh_mesh_viz.points.push_back(
            GtsamToRos(graph_values.at<gtsam::Pose3>(front)).position);
        mesh_mesh_viz.points.push_back(
            GtsamToRos(graph_values.at<gtsam::Pose3>(back)).position);
        color.r = 1.0;
        color.g = 0.0;
        color.b = 0.0;
        color.a = 0.8;
        mesh_mesh_viz.colors.push_back(color);
        mesh_mesh_viz.colors.push_back(color);
      } else {
        // pose-to-mesh
        pose_mesh_viz.points.push_back(
            GtsamToRos(graph_values.at<gtsam::Pose3>(front)).position);
        pose_mesh_viz.points.push_back(
            GtsamToRos(graph_values.at<gtsam::Pose3>(back)).position);
        color.r = 1.0;
        color.g = 1.0;
        color.b = 0.2;
        color.a = 0.3;
        pose_mesh_viz.colors.push_back(color);
        pose_mesh_viz.colors.push_back(color);
      }
    }
    // Publish the msg with the edges
    mesh_mesh_pub->publish(mesh_mesh_viz);
    pose_mesh_pub->publish(pose_mesh_viz);
  }
}

Path KimeraPgmoInterface::getOptimizedTrajectory(const size_t& robot_id) const {
  // return the optimized trajectory (pose graph)
  const char& robot_prefix = robot_id_to_prefix.at(robot_id);
  Path optimized_traj;
  if (run_mode_ == RunMode::DPGMO) {
    size_t n = deformation_graph_->getOptimizedTrajectory(robot_prefix).size();
    for (size_t i = 0; i < n; i++) {
      gtsam::Symbol node(robot_prefix, i);
      if (dpgmo_values_.exists(node)) {
        optimized_traj.push_back(dpgmo_values_.at<gtsam::Pose3>(node));
      } else {
        break;
      }
    }
  } else {
    optimized_traj = deformation_graph_->getOptimizedTrajectory(robot_prefix);
  }
  return optimized_traj;
}

bool KimeraPgmoInterface::loadGraphAndMesh(const size_t& robot_id,
                                           const std::string& ply_path,
                                           const std::string& dgrf_path,
                                           pcl::PolygonMesh::Ptr optimized_mesh,
                                           std::vector<ros::Time>* mesh_vertex_stamps,
                                           bool do_optimize) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  kimera_pgmo::ReadMeshWithStampsFromPly(ply_path, mesh, mesh_vertex_stamps);

  KimeraPgmoMesh mesh_msg =
      PolygonMeshToPgmoMeshMsg(robot_id, *mesh, *mesh_vertex_stamps, "world");

  loadDeformationGraphFromFile(dgrf_path);
  ROS_INFO(
      "Loaded new graph. Currently have %d vertices in deformatio graph and %d loop "
      "closures. ",
      deformation_graph_->getNumVertices(),
      num_loop_closures_);
  return optimizeFullMesh(mesh_msg, optimized_mesh, mesh_vertex_stamps, do_optimize);
}

}  // namespace kimera_pgmo
