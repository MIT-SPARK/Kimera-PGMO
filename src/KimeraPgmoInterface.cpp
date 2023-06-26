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

template <typename T>
bool pgmoParseParam(const ros::NodeHandle& nh,
                    const std::string& name,
                    T& value,
                    bool required) {
  if (!nh.getParam(name, value) && required) {
    ROS_ERROR_STREAM("Failed to find required param " << name);
    return false;
  }
  return true;
}

bool KimeraPgmoConfig::load(const ros::NodeHandle& nh) {
  valid = true;

  int run_mode_num;
  valid &= pgmoParseParam(nh, "run_mode", run_mode_num, true);
  mode = static_cast<RunMode>(run_mode_num);
  if (mode < RunMode::FULL || mode > RunMode::DPGMO) {
    ROS_ERROR_STREAM("Invalid run mode: " << run_mode_num);
    valid = false;
  }

  valid &= pgmoParseParam(nh, "embed_trajectory_delta_t", embed_delta_t, true);
  valid &= pgmoParseParam(nh, "num_interp_pts", num_interp_pts, true);
  if (num_interp_pts <= 1) {
    ROS_ERROR_STREAM("Need at least 2 interpolation points" << num_interp_pts);
    valid = false;
  }
  valid &= pgmoParseParam(nh, "interp_horizon", interp_horizon, true);
  valid &= pgmoParseParam(nh, "add_initial_prior", b_add_initial_prior, true);
  valid &= pgmoParseParam(nh, "covariance/odom", odom_variance, true);
  valid &= pgmoParseParam(nh, "covariance/loop_close", lc_variance, true);
  valid &= pgmoParseParam(nh, "covariance/prior", prior_variance, true);
  valid &= pgmoParseParam(nh, "covariance/mesh_mesh", mesh_edge_variance, true);
  valid &= pgmoParseParam(nh, "covariance/pose_mesh", pose_mesh_variance, true);
  valid &= pgmoParseParam(nh, "rpgo/odom_trans_threshold", odom_trans_threshold, true);
  valid &= pgmoParseParam(nh, "rpgo/odom_rot_threshold", odom_rot_threshold, true);
  valid &= pgmoParseParam(nh, "rpgo/pcm_trans_threshold", pcm_trans_threshold, true);
  valid &= pgmoParseParam(nh, "rpgo/pcm_rot_threshold", pcm_rot_threshold, true);
  valid &= pgmoParseParam(nh, "rpgo/gnc_alpha", gnc_alpha, true);
  valid &= pgmoParseParam(nh, "enable_sparsify", b_enable_sparsify, true);

  pgmoParseParam(nh, "rpgo/gnc_max_iterations", gnc_max_it, false);
  pgmoParseParam(nh, "rpgo/gnc_mu_step", gnc_mu_step, false);
  pgmoParseParam(nh, "rpgo/gnc_cost_tolerance", gnc_cost_tol, false);
  pgmoParseParam(nh, "rpgo/gnc_weight_tolerance", gnc_weight_tol, false);
  pgmoParseParam(nh, "rpgo/gnc_fix_prev_inliers", gnc_fix_prev_inliers, false);
  pgmoParseParam(nh, "rpgo/lm_diagonal_damping", lm_diagonal_damping, false);
  pgmoParseParam(nh, "trans_node_dist", trans_sparse_dist, false);
  pgmoParseParam(nh, "rot_node_dist", rot_sparse_dist, false);
  pgmoParseParam(nh, "output_prefix", log_path, false);

  return valid;
}

KimeraRPGO::RobustSolverParams KimeraPgmoConfig::getRobustSolverParams() const {
  KimeraRPGO::RobustSolverParams params;
  params.setPcmSimple3DParams(odom_trans_threshold,
                              odom_rot_threshold,
                              pcm_trans_threshold,
                              pcm_rot_threshold,
                              KimeraRPGO::Verbosity::UPDATE);
  params.setLmDiagonalDamping(lm_diagonal_damping);

  // Use GNC (confidence value)
  if (gnc_alpha > 0 && gnc_alpha < 1) {
    params.setGncInlierCostThresholdsAtProbability(gnc_alpha,
                                                   static_cast<size_t>(gnc_max_it),
                                                   gnc_mu_step,
                                                   gnc_cost_tol,
                                                   gnc_weight_tol,
                                                   gnc_fix_prev_inliers);
  }

  // Log output
  if (!log_path.empty()) {
    params.logOutput(log_path);
  }

  return params;
}

void SparseKeyframe::initialize(const gtsam::Key& sparse_key,
                                const size_t& robot_id,
                                const size_t& pose_id) {
  const gtsam::Symbol key_symb(GetRobotPrefix(robot_id), pose_id);
  key = sparse_key;
  keyed_transforms.insert({key_symb, gtsam::Pose3()});
}

bool SparseKeyframe::addNewEdge(const pose_graph_tools::PoseGraphEdge& new_edge,
                                double trans_threshold,
                                double rot_threshold) {
  if (!active) {
    ROS_ERROR("Cannot add edge to inactive sparse key-frame.");
    return false;
  }
  if (new_edge.type != pose_graph_tools::PoseGraphEdge::ODOM) {
    ROS_ERROR("Trying to add non-ODOM edge to sparse key-frame.");
    return false;
  }

  const gtsam::Symbol from_key(GetRobotPrefix(new_edge.robot_from), new_edge.key_from);
  const gtsam::Symbol to_key(GetRobotPrefix(new_edge.robot_to), new_edge.key_to);

  if (keyed_transforms.rbegin()->first != to_key - 1) {
    ROS_ERROR("Attempting to insert non-consecutive nodes to sparse key-frame");
    return false;
  }
  const gtsam::Pose3& measure = RosToGtsam(new_edge.pose);
  current_transform = current_transform.compose(measure);
  edges.push_back(new_edge);

  double dist_total = current_transform.translation().norm();
  double rot_total = current_transform.rotation().xyz().norm();
  if (dist_total > trans_threshold || rot_total > rot_threshold) {
    active = false;
    return false;
  }

  keyed_transforms.insert({to_key, current_transform});
  return true;
}

// Constructor
KimeraPgmoInterface::KimeraPgmoInterface()
    : deformation_graph_(new DeformationGraph),
      full_mesh_updated_(false),
      num_loop_closures_(0) {}

KimeraPgmoInterface::~KimeraPgmoInterface() {}

// Load deformation parameters
bool KimeraPgmoInterface::loadParameters(const ros::NodeHandle& n) {
  if (!config_.load(n)) {
    return false;
  }

  return initializeFromConfig();
}

bool KimeraPgmoInterface::initializeFromConfig() {
  if (!config_.valid) {
    return false;
  }

  const auto params = config_.getRobustSolverParams();
  if (!deformation_graph_->initialize(params)) {
    ROS_ERROR("KimeraPgmo: Failed to initialize deformation graph.");
    return false;
  }

  // If inliers are not fixed, need to perform interpolation on whole mesh
  // everytime we optimize
  deformation_graph_->setForceRecalculate(!config_.gnc_fix_prev_inliers);

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

ProcessPoseGraphStatus KimeraPgmoInterface::processIncrementalPoseGraph(
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

    // Initiate first node and add prior
    deformation_graph_->addNewNode(
        key_symb.key(), init_pose, config_.b_add_initial_prior, config_.prior_variance);

    // Create first sparse frame
    SparseKeyframe init_sparse_frame;
    init_sparse_frame.initialize(key_symb, msg->nodes[0].robot_id, msg->nodes[0].key);
    sparse_frames_.insert({key_symb, init_sparse_frame});
    full_sparse_frame_map_.insert({key_symb, key_symb});
    keyed_stamps_.insert({key_symb, msg->nodes[0].header.stamp});

    // Add to trajectory and timestamp map
    initial_trajectory->push_back(init_pose);
    node_timestamps->push_back(msg->nodes[0].header.stamp);

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
          return ProcessPoseGraphStatus::INVALID;
        }

        if (!full_sparse_frame_map_.count(from_key)) {
          ROS_ERROR("Missing from node %s in odometry edge.",
                    gtsam::DefaultKeyFormatter(from_key).c_str());
          return ProcessPoseGraphStatus::MISSING;
        }
        if (full_sparse_frame_map_.count(from_key) && full_sparse_frame_map_.count(to_key)) {
          ROS_WARN("Duplicated edge. ");
          continue;
        }
        gtsam::Key sparse_key = full_sparse_frame_map_.at(from_key);
        bool add_to_sparse_frame = sparse_frames_[sparse_key].addNewEdge(
            pg_edge, config_.trans_sparse_dist, config_.rot_sparse_dist);
        if (add_to_sparse_frame && config_.b_enable_sparsify) {
          full_sparse_frame_map_.insert({to_key, sparse_key});
          keyed_stamps_.insert({to_key, pg_edge.header.stamp});
          node_timestamps->back() = pg_edge.header.stamp;
        } else {
          sparse_frames_[sparse_key].active = false;
          sparse_key = sparse_key + 1;
          full_sparse_frame_map_.insert({to_key, sparse_key});
          keyed_stamps_.insert({to_key, pg_edge.header.stamp});
          SparseKeyframe new_sparse_frame;
          new_sparse_frame.initialize(sparse_key, robot_from, current_node);
          sparse_frames_[sparse_key] = new_sparse_frame;

          const Vertex current_sparse_node = gtsam::Symbol(sparse_key).index();
          if (initial_trajectory->size() != current_sparse_node) {
            ROS_WARN(
                "New current node does not match current trajectory length. %d "
                "vs %d",
                static_cast<int>(initial_trajectory->size()),
                static_cast<int>(current_sparse_node));
            if (initial_trajectory->size() < current_sparse_node) {
              return ProcessPoseGraphStatus::MISSING;
            }
            // duplicate, continue to next edge
            ROS_WARN("Duplicate edge. ");
            continue;
          }
          // Calculate pose of new node
          const gtsam::Pose3& new_pose =
              initial_trajectory->at(current_sparse_node - 1)
                  .compose(sparse_frames_[sparse_key - 1].current_transform);
          // Add to trajectory and timestamp maps
          if (initial_trajectory->size() == current_sparse_node)
            initial_trajectory->push_back(new_pose);
          node_timestamps->push_back(pg_edge.header.stamp);
          // Add new node to queue to be connected to mesh later
          unconnected_nodes->push(current_sparse_node);

          // Add the pose estimate of new node and between factor (odometry) to
          // deformation graph
          deformation_graph_->addNewBetween(
              sparse_key - 1,
              sparse_key,
              sparse_frames_[sparse_key - 1].current_transform,
              new_pose,
              config_.odom_variance);
        }
      } else if (pg_edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE &&
                 config_.mode == RunMode::FULL) {
        if (!full_sparse_frame_map_.count(from_key) ||
            !full_sparse_frame_map_.count(to_key)) {
          ROS_ERROR("Caught loop closure between unknown nodes.");
          return ProcessPoseGraphStatus::LC_MISSING_NODES;
        }
        gtsam::Key from_sparse_key = full_sparse_frame_map_.at(from_key);
        gtsam::Key to_sparse_key = full_sparse_frame_map_.at(to_key);
        // measure = from_T_to. sparse_from_T_sparse_to = sparse_from_T_from * from_T_to
        // * (sparse_to_T_to)^(-1)
        if (loop_closures_.count(from_sparse_key) &&
            loop_closures_.at(from_sparse_key).count(to_sparse_key)) {
          // Loop closure already exists TODO(yun) add flag to toggle this check
          continue;
        }

        gtsam::Pose3 from_sparse_T_to_sparse =
            sparse_frames_.at(from_sparse_key).keyed_transforms.at(from_key) * measure *
            sparse_frames_.at(to_sparse_key).keyed_transforms.at(to_key).inverse();
        // Loop closure edge (only add if we are in full
        // optimization mode ) Add to deformation graph
        deformation_graph_->addNewBetween(from_sparse_key,
                                          to_sparse_key,
                                          from_sparse_T_to_sparse,
                                          gtsam::Pose3(),
                                          config_.lc_variance);
        if (!loop_closures_.count(from_sparse_key)) {
          loop_closures_[from_sparse_key] = std::set<gtsam::Key>();
        }
        if (!loop_closures_.count(to_sparse_key)) {
          loop_closures_[to_sparse_key] = std::set<gtsam::Key>();
        }
        loop_closures_[from_sparse_key].insert(to_sparse_key);
        loop_closures_[to_sparse_key].insert(from_sparse_key);
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
    ROS_ERROR_STREAM("Error in KimeraPgmo incrementalPoseGraphCallback: " << e.what());
    return ProcessPoseGraphStatus::UNKNOWN;
  }
  return ProcessPoseGraphStatus::SUCCESS;
}

void KimeraPgmoInterface::processOptimizedPath(const nav_msgs::Path::ConstPtr& path_msg,
                                               const size_t& robot_id) {
  ROS_INFO("KimeraPgmo: Received optimized trajectory. Fixing pose graph nodes...");

  deformation_graph_->removePriorsWithPrefix(GetRobotPrefix(robot_id));

  std::vector<std::pair<gtsam::Key, gtsam::Pose3> > node_estimates;
  for (size_t i = 0; i < path_msg->poses.size(); i++) {
    node_estimates.push_back(std::pair<gtsam::Key, gtsam::Pose3>(
        gtsam::Symbol(GetRobotPrefix(robot_id), i).key(),
        RosToGtsam(path_msg->poses[i].pose)));
  }
  deformation_graph_->addNodeMeasurements(node_estimates, config_.prior_variance);
}

bool KimeraPgmoInterface::optimizeFullMesh(const KimeraPgmoMesh& mesh_msg,
                                           pcl::PolygonMesh::Ptr optimized_mesh,
                                           std::vector<ros::Time>* mesh_vertex_stamps,
                                           bool do_optimize) {
  std::vector<int> mesh_vertex_graph_inds;
  const pcl::PolygonMesh& input_mesh =
      PgmoMeshMsgToPolygonMesh(mesh_msg, mesh_vertex_stamps, &mesh_vertex_graph_inds);
  // check if empty
  if (input_mesh.cloud.height * input_mesh.cloud.width == 0) return false;

  size_t robot_id = mesh_msg.id;

  std_msgs::Header mesh_header = mesh_msg.header;

  // Optimize mesh
  try {
    if (config_.mode == RunMode::DPGMO) {
      // Here we are getting the optimized values from the dpgo solver
      *optimized_mesh = deformation_graph_->deformMesh(input_mesh,
                                                       *mesh_vertex_stamps,
                                                       mesh_vertex_graph_inds,
                                                       GetVertexPrefix(robot_id),
                                                       dpgmo_values_,
                                                       config_.num_interp_pts,
                                                       config_.interp_horizon);
    } else {
      if (do_optimize) {
        deformation_graph_->optimize();
      }
      *optimized_mesh = deformation_graph_->deformMesh(input_mesh,
                                                       *mesh_vertex_stamps,
                                                       mesh_vertex_graph_inds,
                                                       GetVertexPrefix(robot_id),
                                                       config_.num_interp_pts,
                                                       config_.interp_horizon);
    }
  } catch (const std::out_of_range& e) {
    ROS_ERROR("Failed to deform mesh. Out of range error. ");
    return false;
  }

  full_mesh_updated_ = true;
  return true;
}

ProcessMeshGraphStatus KimeraPgmoInterface::processIncrementalMeshGraph(
    const pose_graph_tools::PoseGraph::ConstPtr& mesh_graph_msg,
    const std::vector<ros::Time>& node_timestamps,
    std::queue<size_t>* unconnected_nodes) {
  if (mesh_graph_msg->edges.size() == 0 || mesh_graph_msg->nodes.size() == 0) {
    ROS_DEBUG("processIncrementalMeshGraph: 0 nodes or 0 edges in mesh graph msg. ");
    return ProcessMeshGraphStatus::EMPTY;
  }
  // Assume graph only contains message from one robot
  size_t robot_id = mesh_graph_msg->nodes[0].robot_id;
  // Mesh edges and nodes
  std::vector<std::pair<gtsam::Key, gtsam::Key> > new_mesh_edges;
  gtsam::Values new_mesh_nodes;
  std::unordered_map<gtsam::Key, ros::Time> new_mesh_node_stamps;
  std::vector<size_t> new_indices;
  std::vector<ros::Time> new_index_stamps;

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
      return ProcessMeshGraphStatus::INVALID;
    }
    gtsam::Key key = gtsam::Symbol(GetVertexPrefix(n.robot_id), n.key);
    gtsam::Pose3 node_pose = RosToGtsam(n.pose);
    try {
      new_mesh_nodes.insert(key, node_pose);
      new_mesh_node_stamps.insert({key, n.header.stamp});
    } catch (const gtsam::ValuesKeyAlreadyExists& e) {
      ROS_WARN("processing mesh node duplicate\n");
    }
  }

  // Add to deformation graph
  deformation_graph_->addNewMeshEdgesAndNodes(new_mesh_edges,
                                              new_mesh_nodes,
                                              new_mesh_node_stamps,
                                              &new_indices,
                                              &new_index_stamps,
                                              config_.mesh_edge_variance);
  assert(new_indices.size() == new_index_stamps.size());
  bool connection = false;
  if (!unconnected_nodes->empty() && new_indices.size() > 0) {
    std::map<size_t, std::vector<size_t>> node_valences;
    for (size_t i = 0; i < new_indices.size(); i++) {
      size_t idx = new_indices[i];
      double idx_time = new_index_stamps[i].toSec();
      size_t closest_node = unconnected_nodes->front();
      double min_difference = std::numeric_limits<double>::infinity();
      while (!unconnected_nodes->empty()) {
        const size_t node = unconnected_nodes->front();
        if (abs(node_timestamps[node].toSec() - idx_time) < min_difference) {
          min_difference = abs(node_timestamps[node].toSec() - idx_time);
          closest_node = node;
          if (unconnected_nodes->size() == 1) {
            break;
          }
          unconnected_nodes->pop();
        } else {
          break;
        }
      }
      if (!node_valences.count(closest_node)) {
        node_valences.insert({closest_node, std::vector<size_t>()});
      }
      node_valences[closest_node].push_back(idx);
    }

    for (const auto& node_val : node_valences) {
      if (verbose_) {
        ROS_INFO("Connecting robot %d node %d to %d vertices. ",
                 static_cast<int>(robot_id),
                 static_cast<int>(node_val.first),
                 static_cast<int>(node_val.second.size()));
      }
      deformation_graph_->addNodeValence(
          gtsam::Symbol(GetRobotPrefix(robot_id), node_val.first),
          node_val.second,
          GetVertexPrefix(robot_id),
          config_.pose_mesh_variance);
      connection = true;
    }
  }
  if (!connection && new_indices.size() > 0) {
    ROS_WARN("KimeraPgmo: Partial mesh not connected to pose graph. ");
  }

  return ProcessMeshGraphStatus::SUCCESS;
}

bool KimeraPgmoInterface::saveMesh(const pcl::PolygonMesh& mesh,
                                   const std::string& ply_name) {
  // Save mesh
  WriteMeshToPly(ply_name, mesh);
  ROS_INFO("KimeraPgmo: Saved mesh to file.");
  return true;
}

bool KimeraPgmoInterface::saveTrajectory(const Path& trajectory,
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
    csvfile << stamp.toNSec() << "," << pos.x() << "," << pos.y() << "," << pos.z()
            << "," << quat.w() << "," << quat.x() << "," << quat.y() << "," << quat.z()
            << "\n";
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

bool KimeraPgmoInterface::savePoseGraphSparseMapping(const std::string& output_path) {
  std::ofstream outfile;
  outfile.open(output_path);
  outfile << "full-key,sparse-key,x,y,z,qw,qx,qy,qz\n";
  // TODO this doesn't save all infromation of the sparse frames
  for (const auto& mapping : full_sparse_frame_map_) {
    outfile << std::to_string(mapping.first) << "," << std::to_string(mapping.second);
    const auto& sparse_frame = sparse_frames_.at(mapping.second);
    const gtsam::Pose3& rel_pose = sparse_frame.keyed_transforms.at(mapping.first);
    const gtsam::Point3& pos = rel_pose.translation();
    const gtsam::Quaternion& quat = rel_pose.rotation().toQuaternion();
    outfile << "," << pos.x() << "," << pos.y() << "," << pos.z() << "," << quat.w()
            << "," << quat.x() << "," << quat.y() << "," << quat.z() << "\n";
  }
  outfile.close();
  return true;
}

bool KimeraPgmoInterface::loadPoseGraphSparseMapping(const std::string& input_path) {
  std::ifstream infile(input_path);

  // Values that will be filled
  gtsam::Key full_key, sparse_key;
  double qx, qy, qz, qw;
  double tx, ty, tz;

  std::string line;
  std::string token;
  // Skip first line (headers)
  std::getline(infile, line);
  // Iterate over remaining lines
  while (std::getline(infile, line)) {
    std::istringstream ss(line);

    std::getline(ss, token, ',');
    full_key = std::stoull(token);
    std::getline(ss, token, ',');
    sparse_key = std::stoull(token);

    std::getline(ss, token, ',');
    tx = std::stod(token);
    std::getline(ss, token, ',');
    ty = std::stod(token);
    std::getline(ss, token, ',');
    tz = std::stod(token);

    std::getline(ss, token, ',');
    qw = std::stod(token);
    std::getline(ss, token, ',');
    qx = std::stod(token);
    std::getline(ss, token, ',');
    qy = std::stod(token);
    std::getline(ss, token, ',');
    qz = std::stod(token);

    full_sparse_frame_map_.insert({full_key, sparse_key});
    if (sparse_frames_.count(sparse_key) == 0) {
      sparse_frames_.insert({sparse_key, SparseKeyframe()});
    }
    gtsam::Pose3 transform =
        gtsam::Pose3(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));
    sparse_frames_[sparse_key].keyed_transforms.insert({full_key, transform});
  }
  infile.close();
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
            deformation_graph_->getInitialPositionVertex(vertex_prefix, pg_edge.key_to);
        pg_edge.pose =
            GtsamToRos(gtsam::Pose3(gtsam::Rot3(), vertex_pos_to - vertex_pos_from));

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
            deformation_graph_->getInitialPositionVertex(vertex_prefix, pg_edge.key_to);
        pg_edge.pose =
            GtsamToRos(gtsam::Pose3(gtsam::Rot3(),
                                    pose_from.rotation().inverse().rotate(
                                        vertex_pos_to - pose_from.translation())));

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
        pg_edge.pose = GtsamToRos(
            gtsam::Pose3(gtsam::Rot3(), pose_to.translation() - vertex_pos_from));

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
    pg_node.pose = GtsamToRos(gtsam::Pose3(gtsam::Rot3(), initial_positions[i]));
    pg_mesh_msg->nodes.push_back(pg_node);
  }
  return true;
}

void KimeraPgmoInterface::visualizeDeformationGraphMeshEdges(
    const ros::Publisher* mesh_mesh_pub,
    const ros::Publisher* pose_mesh_pub) const {
  if (mesh_mesh_pub->getNumSubscribers() > 0 ||
      pose_mesh_pub->getNumSubscribers() > 0) {
    const ros::Time curr_time = ros::Time::now();
    visualization_msgs::Marker mesh_mesh_viz;
    visualization_msgs::Marker pose_mesh_viz;
    fillDeformationGraphMarkers(
        *deformation_graph_, curr_time, mesh_mesh_viz, pose_mesh_viz);

    // Publish the msg with the edges
    mesh_mesh_pub->publish(mesh_mesh_viz);
    pose_mesh_pub->publish(pose_mesh_viz);
  }
}

Path KimeraPgmoInterface::getOptimizedTrajectory(const size_t& robot_id) const {
  // return the optimized trajectory (pose graph)
  const char& robot_prefix = robot_id_to_prefix.at(robot_id);
  Path optimized_traj;
  if (config_.mode == RunMode::DPGMO) {
    size_t n = deformation_graph_->getQueuedTrajectory(robot_prefix).size();
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
  if (!config_.b_enable_sparsify) {
    // The optimized trajectory in deformation graph is already the full trajectory
    return optimized_traj;
  }
  Path optimized_traj_interpolated;
  for (size_t i = 0; i < optimized_traj.size(); i++) {
    gtsam::Key sparse_key = gtsam::Symbol(robot_prefix, i);
    for (const auto& keyed_pose : sparse_frames_.at(sparse_key).keyed_transforms) {
      optimized_traj_interpolated.push_back(
          optimized_traj[i].compose(keyed_pose.second));
    }
  }
  return optimized_traj_interpolated;
}

std::vector<ros::Time> KimeraPgmoInterface::getRobotTimestamps(
    const size_t& robot_id) const {
  std::vector<ros::Time> stamps;
  const char& robot_prefix = robot_id_to_prefix.at(robot_id);
  // TODO(yun) remove call to get optimized trajectory
  Path optimized_traj = deformation_graph_->getOptimizedTrajectory(robot_prefix);
  for (size_t i = 0; i < optimized_traj.size(); i++) {
    gtsam::Key sparse_key = gtsam::Symbol(robot_prefix, i);
    for (const auto& keyed_pose : sparse_frames_.at(sparse_key).keyed_transforms) {
      stamps.push_back(keyed_stamps_.at(keyed_pose.first));
    }
  }
  return stamps;
}

bool KimeraPgmoInterface::loadGraphAndMesh(const size_t& robot_id,
                                           const std::string& ply_path,
                                           const std::string& dgrf_path,
                                           const std::string& sparse_mapping_file_path,
                                           pcl::PolygonMesh::Ptr optimized_mesh,
                                           std::vector<ros::Time>* mesh_vertex_stamps,
                                           bool do_optimize) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  kimera_pgmo::ReadMeshWithStampsFromPly(ply_path, mesh, mesh_vertex_stamps);

  KimeraPgmoMesh mesh_msg =
      PolygonMeshToPgmoMeshMsg(robot_id, *mesh, *mesh_vertex_stamps, "world");

  loadPoseGraphSparseMapping(sparse_mapping_file_path);
  loadDeformationGraphFromFile(dgrf_path);
  ROS_INFO_STREAM("Loaded new graph. Currently have "
                  << deformation_graph_->getNumVertices()
                  << "vertices in deformation graph and " << num_loop_closures_
                  << " loop closures.");
  return optimizeFullMesh(mesh_msg, optimized_mesh, mesh_vertex_stamps, do_optimize);
}

}  // namespace kimera_pgmo
