/**
 * @file   kimera_pgmo_interface.cpp
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#include "kimera_pgmo/kimera_pgmo_interface.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <optional>

#include "kimera_pgmo/utils/mesh_io.h"

namespace kimera_pgmo {

using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;

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

void declare_config(KimeraPgmoConfig& config) {
  using namespace config;
  name("KimeraPgmoConfig");
  enum_field(
      config.mode,
      "run_mode",
      {{RunMode::FULL, "FULL"}, {RunMode::MESH, "MESH"}, {RunMode::DPGMO, "DPGMO"}});
  field(config.embed_delta_t, "embed_trajectory_delta_t");
  field(config.num_interp_pts, "num_interp_pts");
  field(config.interp_horizon, "interp_horizon");
  field(config.b_add_initial_prior, "add_initial_prior");
  field(config.log_path, "output_prefix");
  field(config.b_enable_sparsify, "enable_sparsify");
  field(config.trans_sparse_dist, "trans_node_dist");
  field(config.rot_sparse_dist, "rot_node_dist");

  {  // config namespace covariance
    NameSpace ns("covariance");
    field(config.odom_variance, "odom");
    field(config.lc_variance, "loop_close");
    field(config.prior_variance, "prior");
    field(config.mesh_edge_variance, "mesh_mesh");
    field(config.pose_mesh_variance, "pose_mesh");
  }

  {  // config namespace rpgo
    NameSpace ns("rpgo");
    field(config.odom_trans_threshold, "odom_trans_threshold");
    field(config.odom_rot_threshold, "odom_rot_threshold");
    field(config.pcm_trans_threshold, "pcm_trans_threshold");
    field(config.pcm_rot_threshold, "pcm_rot_threshold");
    field(config.gnc_alpha, "gnc_alpha");
    field(config.gnc_max_it, "gnc_max_iterations");
    field(config.gnc_mu_step, "gnc_mu_step");
    field(config.gnc_cost_tol, "gnc_cost_tolerance");
    field(config.gnc_weight_tol, "gnc_weight_tolerance");
    field(config.gnc_fix_prev_inliers, "gnc_fix_prev_inliers");
    field(config.lm_diagonal_damping, "lm_diagonal_damping");
  }

  check(config.num_interp_pts, GT, 1, "num_interp_pts");
}

// Constructor
KimeraPgmoInterface::KimeraPgmoInterface()
    : full_mesh_updated_(false),
      deformation_graph_(new DeformationGraph),
      num_loop_closures_(0) {}

// Load deformation parameters
bool KimeraPgmoInterface::initialize(const KimeraPgmoConfig& config) {
  config_ = config;
  return initializeFromConfig();
}

bool KimeraPgmoInterface::loadGraphAndMesh(size_t robot_id,
                                           const std::string& ply_path,
                                           const std::string& dgrf_path,
                                           const std::string& sparse_mapping_file_path,
                                           pcl::PolygonMesh::Ptr optimized_mesh,
                                           std::vector<Timestamp>* mesh_vertex_stamps,
                                           bool do_optimize) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  kimera_pgmo::ReadMeshWithStampsFromPly(ply_path, mesh, mesh_vertex_stamps);

  loadPoseGraphSparseMapping(sparse_mapping_file_path);
  loadDeformationGraphFromFile(dgrf_path);
  SPARK_LOG(INFO) << "Loaded new graph. Currently have "
                  << deformation_graph_->getNumVertices()
                  << "vertices in deformation graph and " << num_loop_closures_
                  << " loop closures.";

  std::vector<int> fake_indices(mesh_vertex_stamps->size(), 0);
  return optimizeFullMesh(
      robot_id, *mesh, *mesh_vertex_stamps, fake_indices, *optimized_mesh, do_optimize);
}

Path KimeraPgmoInterface::getOptimizedTrajectory(size_t robot_id) const {
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

std::vector<Timestamp> KimeraPgmoInterface::getRobotTimestamps(size_t robot_id) const {
  std::vector<Timestamp> stamps;
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

gtsam::NonlinearFactorGraph KimeraPgmoInterface::getDeformationGraphFactors() const {
  return deformation_graph_->getGtsamFactors();
}

gtsam::Values KimeraPgmoInterface::getDeformationGraphValues() const {
  return deformation_graph_->getGtsamValues();
}

DeformationGraphPtr KimeraPgmoInterface::getDeformationGraphPtr() const {
  return deformation_graph_;
}

bool KimeraPgmoInterface::wasFullMeshUpdated(bool clear_flag) {
  bool to_return = full_mesh_updated_;
  if (clear_flag) {
    full_mesh_updated_ = false;
  }
  return to_return;
}

void KimeraPgmoInterface::forceOptimize() { return deformation_graph_->optimize(); }

void KimeraPgmoInterface::resetDeformationGraph() {
  KimeraRPGO::RobustSolverParams pgo_params = deformation_graph_->getParams();
  deformation_graph_.reset(new DeformationGraph);
  deformation_graph_->initialize(pgo_params);
}

void KimeraPgmoInterface::loadDeformationGraphFromFile(const std::string& input) {
  deformation_graph_->load(input);
  num_loop_closures_ = deformation_graph_->getNumLoopclosures();
}

void KimeraPgmoInterface::loadDeformationGraphFromFile(const std::string& input,
                                                       size_t robot_id) {
  deformation_graph_->load(input, true, true, robot_id);
  num_loop_closures_ = deformation_graph_->getNumLoopclosures();
}

bool KimeraPgmoInterface::initializeFromConfig() {
  if (!config::isValid(config_)) {
    return false;
  }

  const auto params = config_.getRobustSolverParams();
  if (!deformation_graph_->initialize(params)) {
    SPARK_LOG(ERROR) << "KimeraPgmo: Failed to initialize deformation graph.";
    return false;
  }

  // If inliers are not fixed, need to perform interpolation on whole mesh
  // everytime we optimize
  deformation_graph_->setForceRecalculate(!config_.gnc_fix_prev_inliers);
  return true;
}

ProcessPoseGraphStatus KimeraPgmoInterface::processIncrementalPoseGraph(
    const PoseGraph& msg,
    Path& initial_trajectory,
    std::vector<Timestamp>& node_timestamps,
    std::queue<size_t>& unconnected_nodes) {
  // if first node initialize
  //// Note that we assume for all node ids that the keys start with 0
  if (!msg.nodes.empty() && msg.nodes[0].key == 0 && initial_trajectory.empty()) {
    const auto init_node = msg.nodes[0];
    size_t robot_id = init_node.robot_id;

    const gtsam::Symbol key_symb(GetRobotPrefix(robot_id), 0);
    const gtsam::Pose3 init_pose(init_node.pose.matrix());

    // Initiate first node and add prior
    deformation_graph_->addNewNode(
        key_symb.key(), init_pose, config_.b_add_initial_prior, config_.prior_variance);

    // Create first sparse frame
    SparseKeyframe init_sparse_frame;
    init_sparse_frame.initialize(key_symb, init_node.robot_id, init_node.key);
    sparse_frames_.insert({key_symb, init_sparse_frame});
    full_sparse_frame_map_.insert({key_symb, key_symb});
    keyed_stamps_.insert({key_symb, init_node.stamp_ns});

    // Add to trajectory and timestamp map
    initial_trajectory.push_back(init_pose);
    node_timestamps.push_back(init_node.stamp_ns);

    // Push node to queue to be connected to mesh vertices later
    unconnected_nodes.push(0);
    SPARK_LOG(DEBUG) << "Initialized first node in pose graph.";
  }

  try {
    for (const auto& pg_edge : msg.edges) {
      // Get edge information
      const gtsam::Pose3 measure(pg_edge.pose.matrix());
      const Vertex& prev_node = pg_edge.key_from;
      const Vertex& current_node = pg_edge.key_to;

      size_t robot_from = pg_edge.robot_from;
      size_t robot_to = pg_edge.robot_to;

      const gtsam::Symbol from_key(GetRobotPrefix(robot_from), prev_node);
      const gtsam::Symbol to_key(GetRobotPrefix(robot_to), current_node);

      if (pg_edge.type == pose_graph_tools::PoseGraphEdge::ODOM) {
        // odometry edge
        if (robot_from != robot_to) {
          SPARK_LOG(ERROR)
              << "Odometry edge should not connect two nodes from different "
                 "robots.";
          return ProcessPoseGraphStatus::INVALID;
        }

        if (!full_sparse_frame_map_.count(from_key)) {
          SPARK_LOG(ERROR) << "Missing from node "
                           << gtsam::DefaultKeyFormatter(from_key).c_str()
                           << " in odometry edge.";
          return ProcessPoseGraphStatus::MISSING;
        }

        if (full_sparse_frame_map_.count(from_key) &&
            full_sparse_frame_map_.count(to_key)) {
          SPARK_LOG(WARNING) << "Duplicated edge.";
          continue;
        }

        gtsam::Key sparse_key = full_sparse_frame_map_.at(from_key);
        bool add_to_sparse_frame = sparse_frames_[sparse_key].addNewEdge(
            pg_edge, config_.trans_sparse_dist, config_.rot_sparse_dist);

        if (add_to_sparse_frame && config_.b_enable_sparsify) {
          full_sparse_frame_map_.insert({to_key, sparse_key});
          keyed_stamps_.insert({to_key, pg_edge.stamp_ns});
          node_timestamps.back() = pg_edge.stamp_ns;
        } else {
          sparse_frames_[sparse_key].active = false;
          sparse_key = sparse_key + 1;
          full_sparse_frame_map_.insert({to_key, sparse_key});
          keyed_stamps_.insert({to_key, pg_edge.stamp_ns});
          SparseKeyframe new_sparse_frame;
          new_sparse_frame.initialize(sparse_key, robot_from, current_node);
          sparse_frames_[sparse_key] = new_sparse_frame;

          const Vertex current_sparse_node = gtsam::Symbol(sparse_key).index();
          if (initial_trajectory.size() != current_sparse_node) {
            SPARK_LOG(WARNING)
                << "New current node does not match current trajectory length. "
                << initial_trajectory.size() << " vs " << current_sparse_node;
            if (initial_trajectory.size() < current_sparse_node) {
              return ProcessPoseGraphStatus::MISSING;
            }

            // duplicate, continue to next edge
            SPARK_LOG(WARNING) << "Duplicate edge.";
            continue;
          }

          // Calculate pose of new node
          const auto& new_pose =
              initial_trajectory.at(current_sparse_node - 1)
                  .compose(sparse_frames_[sparse_key - 1].current_transform);

          // Add to trajectory and timestamp maps
          if (initial_trajectory.size() == current_sparse_node) {
            initial_trajectory.push_back(new_pose);
          }

          node_timestamps.push_back(pg_edge.stamp_ns);
          // Add new node to queue to be connected to mesh later
          unconnected_nodes.push(current_sparse_node);

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
          SPARK_LOG(ERROR) << "Caught loop closure between unknown nodes.";
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
        SPARK_LOG(INFO) << "KimeraPgmo: Loop closure detected between robot "
                        << robot_from << " node " << prev_node << " and robot "
                        << robot_to << " node " << current_node << ".";
        num_loop_closures_++;
      }
    }
  } catch (const std::exception& e) {
    SPARK_LOG(ERROR) << "Error in KimeraPgmo incrementalPoseGraphCallback: "
                     << e.what();
    return ProcessPoseGraphStatus::UNKNOWN;
  }

  return ProcessPoseGraphStatus::SUCCESS;
}

ProcessMeshGraphStatus KimeraPgmoInterface::processIncrementalMeshGraph(
    const PoseGraph& mesh_graph,
    const std::vector<Timestamp>& node_timestamps,
    std::queue<size_t>& unconnected_nodes) {
  if (mesh_graph.edges.empty() || mesh_graph.nodes.empty()) {
    SPARK_LOG(DEBUG)
        << "processIncrementalMeshGraph: 0 nodes or 0 edges in mesh graph msg.";
    return ProcessMeshGraphStatus::EMPTY;
  }

  // Assume graph only contains message from one robot
  size_t robot_id = mesh_graph.nodes[0].robot_id;
  // Mesh edges and nodes
  std::vector<std::pair<gtsam::Key, gtsam::Key>> new_mesh_edges;
  gtsam::Values new_mesh_nodes;
  std::unordered_map<gtsam::Key, Timestamp> new_mesh_node_stamps;
  std::vector<size_t> new_indices;
  std::vector<Timestamp> new_index_stamps;

  // Convert and add edges
  for (const auto& e : mesh_graph.edges) {
    if (e.robot_from != robot_id || e.robot_to != robot_id) {
      SPARK_LOG(WARNING)
          << "processIncrementalMeshGraph: detect different robot ids in single "
             "mesh graph msg.";
    }

    gtsam::Key from = gtsam::Symbol(GetVertexPrefix(e.robot_from), e.key_from);
    gtsam::Key to = gtsam::Symbol(GetVertexPrefix(e.robot_to), e.key_to);
    new_mesh_edges.push_back(std::pair<gtsam::Key, gtsam::Key>(from, to));
  }

  // Convert and add nodes
  for (const auto& n : mesh_graph.nodes) {
    if (n.robot_id != robot_id) {
      SPARK_LOG(WARNING)
          << "processIncrementalMeshGraph: detect different robot ids in single "
             "mesh graph msg.";
      return ProcessMeshGraphStatus::INVALID;
    }

    gtsam::Key key = gtsam::Symbol(GetVertexPrefix(n.robot_id), n.key);
    gtsam::Pose3 node_pose(n.pose.matrix());
    try {
      new_mesh_nodes.insert(key, node_pose);
      new_mesh_node_stamps.insert({key, n.stamp_ns});
    } catch (const gtsam::ValuesKeyAlreadyExists& e) {
      SPARK_LOG(WARNING) << "processing mesh node duplicate";
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
  if (!unconnected_nodes.empty() && new_indices.size() > 0) {
    std::map<size_t, std::vector<size_t>> node_valences;
    for (size_t i = 0; i < new_indices.size(); i++) {
      size_t idx = new_indices[i];
      Timestamp idx_time = new_index_stamps[i];
      size_t closest_node = unconnected_nodes.front();
      Timestamp min_difference = std::numeric_limits<Timestamp>::max();
      while (!unconnected_nodes.empty()) {
        const size_t node = unconnected_nodes.front();
        if (abs(int64_t(node_timestamps[node] - idx_time)) < min_difference) {
          min_difference = abs(int64_t(node_timestamps[node] - idx_time));
          closest_node = node;
          if (unconnected_nodes.size() == 1) {
            break;
          }

          unconnected_nodes.pop();
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
        SPARK_LOG(INFO) << "Connecting robot " << robot_id << " node " << node_val.first
                        << " to " << node_val.second.size() << " vertices.";
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
    SPARK_LOG(WARNING) << "KimeraPgmo: Partial mesh not connected to pose graph.";
  }

  return ProcessMeshGraphStatus::SUCCESS;
}

void KimeraPgmoInterface::processOptimizedPath(const Path& path, size_t robot_id) {
  SPARK_LOG(INFO)
      << "KimeraPgmo: Received optimized trajectory. Fixing pose graph nodes...";
  deformation_graph_->removePriorsWithPrefix(GetRobotPrefix(robot_id));

  std::vector<std::pair<gtsam::Key, gtsam::Pose3>> node_estimates;
  for (size_t i = 0; i < path.size(); i++) {
    const gtsam::Symbol node_symbol(GetRobotPrefix(robot_id), i);
    node_estimates.push_back({node_symbol.key(), path[i]});
  }

  deformation_graph_->addNodeMeasurements(node_estimates, config_.prior_variance);
}

bool KimeraPgmoInterface::optimizeFullMesh(
    size_t robot_id,
    const pcl::PolygonMesh& input_mesh,
    const std::vector<Timestamp>& mesh_vertex_stamps,
    const std::vector<int> mesh_vertex_graph_inds,
    pcl::PolygonMesh& optimized_mesh,
    bool do_optimize) {
  // check if empty
  if (input_mesh.cloud.height * input_mesh.cloud.width == 0) {
    return false;
  }

  // Optimize mesh
  try {
    if (config_.mode == RunMode::DPGMO) {
      // Here we are getting the optimized values from the dpgo solver
      optimized_mesh = deformation_graph_->deformMesh(input_mesh,
                                                      mesh_vertex_stamps,
                                                      mesh_vertex_graph_inds,
                                                      GetVertexPrefix(robot_id),
                                                      dpgmo_values_,
                                                      config_.num_interp_pts,
                                                      config_.interp_horizon);
    } else {
      if (do_optimize) {
        deformation_graph_->optimize();
      }

      optimized_mesh = deformation_graph_->deformMesh(input_mesh,
                                                      mesh_vertex_stamps,
                                                      mesh_vertex_graph_inds,
                                                      GetVertexPrefix(robot_id),
                                                      config_.num_interp_pts,
                                                      config_.interp_horizon);
    }
  } catch (const std::out_of_range& e) {
    SPARK_LOG(ERROR) << "Failed to deform mesh. Out of range error.";
    return false;
  }

  full_mesh_updated_ = true;
  return true;
}

bool KimeraPgmoInterface::saveMesh(const pcl::PolygonMesh& mesh,
                                   const std::string& ply_name) {
  // Save mesh
  WriteMeshToPly(ply_name, mesh);
  SPARK_LOG(INFO) << "KimeraPgmo: Saved mesh to file.";
  return true;
}

bool KimeraPgmoInterface::saveTrajectory(const Path& trajectory,
                                         const std::vector<Timestamp>& timestamps,
                                         const std::string& csv_file) {
  // There should be a timestamp associated with each pose
  assert(trajectory.size() == timestamps.size());

  std::ofstream csvfile;
  csvfile.open(csv_file);
  csvfile << "timestamp[ns],x,y,z,qw,qx,qy,qz\n";
  for (size_t i = 0; i < trajectory.size(); i++) {
    const gtsam::Point3& pos = trajectory[i].translation();
    const gtsam::Quaternion& quat = trajectory[i].rotation().toQuaternion();
    csvfile << timestamps[i] << "," << pos.x() << "," << pos.y() << "," << pos.z()
            << "," << quat.w() << "," << quat.x() << "," << quat.y() << "," << quat.z()
            << "\n";
  }
  csvfile.close();
  SPARK_LOG(INFO) << "KimeraPgmo: Saved trajectories to file.";
  return true;
}

bool KimeraPgmoInterface::saveDeformationGraph(const std::string& dgrf_name) {
  // Save mesh
  deformation_graph_->save(dgrf_name);
  SPARK_LOG(INFO) << "KimeraPgmo: Saved deformation graph to file.";
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
    const auto& rel_pose = sparse_frame.keyed_transforms.at(mapping.first);
    const auto pos = rel_pose.translation();
    const auto quat = rel_pose.rotation().toQuaternion();
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

    gtsam::Pose3 transform(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));
    sparse_frames_[sparse_key].keyed_transforms.insert({full_key, transform});
  }

  infile.close();
  return true;
}

bool KimeraPgmoInterface::getConsistencyFactors(size_t robot_id,
                                                PoseGraph& pg_mesh_msg,
                                                size_t vertex_index_offset) const {
  pg_mesh_msg.edges.clear();
  pg_mesh_msg.nodes.clear();

  // Make sure that robot id is valid
  if (robot_id_to_prefix.find(robot_id) == robot_id_to_prefix.end()) {
    SPARK_LOG(ERROR) << "Unexpected robot id.";
    return false;
  }

  // Get the edges from the deformation graph
  auto edge_factors = deformation_graph_->getConsistencyFactors();

  // Get the prefixes
  char v_prefix = robot_id_to_vertex_prefix.at(robot_id);
  char r_prefix = robot_id_to_prefix.at(robot_id);

  // Iterate and convert the edges to PoseGraphEdge type
  for (const auto& factor : edge_factors) {
    gtsam::Symbol from(factor->front());
    gtsam::Symbol to(factor->back());

    std::optional<PoseGraphEdge::Type> edge_type;
    if (from.chr() == v_prefix) {
      if (to.chr() == v_prefix) {
        edge_type = PoseGraphEdge::MESH;
      } else if (to.chr() == r_prefix) {
        edge_type = PoseGraphEdge::MESH_POSE;
      }
    } else if (from.chr() == r_prefix) {
      if (to.chr() == v_prefix) {
        edge_type = PoseGraphEdge::POSE_MESH;
      } else if (to.chr() == r_prefix) {
        SPARK_LOG(ERROR)
            << "Getting a pose-to-pose edge in deformation graph consistency "
               "factors. Check for bug.";
        continue;
      }
    }

    if (!edge_type) {
      SPARK_LOG(WARNING) << "Unexpected edge type.";
      continue;
    }

    auto& pg_edge = pg_mesh_msg.edges.emplace_back();
    pg_edge.key_from = from.index();
    pg_edge.key_to = to.index();
    pg_edge.robot_from = robot_id;
    pg_edge.robot_to = robot_id;
    pg_edge.type = edge_type.value();
    // Covariance is infinite for rotation part
    pg_edge.covariance(3, 3) = 1.0 / 0.0;
    pg_edge.covariance(4, 4) = 1.0 / 0.0;
    pg_edge.covariance(5, 5) = 1.0 / 0.0;

    // Pose should be [I , R_1^{-1} (t2 - t1)] *** these are all initial
    // poses/positions
    switch (pg_edge.type) {
      case PoseGraphEdge::MESH: {
        const auto& p_from =
            deformation_graph_->getInitialPositionVertex(v_prefix, pg_edge.key_from);
        const auto& p_to =
            deformation_graph_->getInitialPositionVertex(v_prefix, pg_edge.key_to);
        pg_edge.pose = Eigen::Translation<double, 3>(p_to - p_from);
        // Update key with offset
        pg_edge.key_from = pg_edge.key_from + vertex_index_offset;
        pg_edge.key_to = pg_edge.key_to + vertex_index_offset;
        break;
      }
      case PoseGraphEdge::POSE_MESH: {
        const auto& pose_from =
            deformation_graph_->getInitialPose(r_prefix, pg_edge.key_from);
        const auto& p_to =
            deformation_graph_->getInitialPositionVertex(v_prefix, pg_edge.key_to);
        const Eigen::Vector3d p_diff =
            pose_from.rotation().inverse().rotate(p_to - pose_from.translation());
        pg_edge.pose = Eigen::Translation<double, 3>(p_diff);

        // Update key with offset
        pg_edge.key_to = pg_edge.key_to + vertex_index_offset;
        break;
      }
      case PoseGraphEdge::MESH_POSE: {
        const auto& p_from =
            deformation_graph_->getInitialPositionVertex(v_prefix, pg_edge.key_from);
        const auto& pose_to =
            deformation_graph_->getInitialPose(r_prefix, pg_edge.key_to);
        pg_edge.pose = Eigen::Translation<double, 3>(pose_to.translation() - p_from);

        // Update key with offset
        pg_edge.key_from = pg_edge.key_from + vertex_index_offset;
        break;
      }
      default:
        SPARK_LOG(ERROR) << "Uknown edge type!";
        break;
    }
  }

  if (pg_mesh_msg.edges.size() == 0) {
    return false;
  }

  // Get the nodes from the deformation graph
  const auto& p_initial = deformation_graph_->getInitialPositionsVertices(v_prefix);
  for (size_t i = 0; i < p_initial.size(); i++) {
    auto& pg_node = pg_mesh_msg.nodes.emplace_back();
    pg_node.robot_id = robot_id;
    pg_node.key = i + vertex_index_offset;
    pg_node.pose = Eigen::Translation<double, 3>(p_initial[i]);
  }

  return true;
}

void KimeraPgmoInterface::insertDpgmoValues(const gtsam::Key& key,
                                            const gtsam::Pose3& pose) {
  if (dpgmo_values_.exists(key)) {
    SPARK_LOG(ERROR) << "Attempting to insert existing key to dpgmo values.";
    return;
  }

  dpgmo_values_.insert(key, pose);
}

/*! \brief Get the DPGMO optimized values
 */
gtsam::Values KimeraPgmoInterface::getDpgmoValues() const { return dpgmo_values_; }

void KimeraPgmoInterface::setVerboseFlag(bool verbose) {
  verbose_ = verbose;
  if (deformation_graph_) {
    deformation_graph_->setVerboseFlag(verbose);
  }
}

}  // namespace kimera_pgmo
