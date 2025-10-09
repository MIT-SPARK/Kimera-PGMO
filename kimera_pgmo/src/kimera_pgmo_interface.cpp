/**
 * @file   kimera_pgmo_interface.cpp
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#include "kimera_pgmo/kimera_pgmo_interface.h"

#include <config_utilities/config.h>
#include <config_utilities/printing.h>
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

using BetweenFactorT = gtsam::BetweenFactor<gtsam::Pose3>;

std::ostream& operator<<(std::ostream& out, const OptimizeStats& stats) {
  out << "total_factors: " << stats.total_factors
      << "\ntotal_values: " << stats.total_values
      << "\ntotal_loop_closures: " << stats.total_loop_closures
      << "\ninlier_loop_closures: " << stats.inlier_loop_closures
      << "\ntotal_interrobot_loop_closures: " << stats.total_interrobot_loop_closures
      << "\ninlier_interrobot_loop_closures: " << stats.inlier_interrobot_loop_closures
      << "\nelapsed_s: " << stats.elapsed_s;
  return out;
}

void declare_config(KimeraPgmoConfig& config) {
  using namespace config;
  name("KimeraPgmoConfig");
  enum_field(config.mode,
             "run_mode",
             {{RunMode::FULL, "FULL"},
              {RunMode::EXTERNAL_OPTIMIZER, "EXTERNAL_OPTIMIZER"},
              {RunMode::MESH_ONLY, "MESH_ONLY"}});
  field(config.num_interp_pts, "num_interp_pts");
  field(config.interp_horizon, "interp_horizon");
  field(config.b_add_initial_prior, "add_initial_prior");
  field(config.log_path, "output_prefix");

  {  // config namespace covariance
    NameSpace ns("covariance");
    field(config.odom_variance, "odom");
    field(config.lc_variance, "loop_close");
    field(config.prior_variance, "prior");
    field(config.mesh_edge_variance, "mesh_mesh");
    field(config.pose_mesh_variance, "pose_mesh");
  }

  field(config.optimizer, "optimizer");

  check(config.num_interp_pts, GT, 1, "num_interp_pts");
  check(config.interp_horizon, GT, 0.0, "interp_horizon");
  check(config.odom_variance, GT, 0.0, "odom_variance");
  check(config.lc_variance, GT, 0.0, "lc_variance");
  check(config.prior_variance, GT, 0.0, "prior_variance");
  check(config.mesh_edge_variance, GT, 0.0, "mesh_edge_variance");
  check(config.pose_mesh_variance, GT, 0.0, "pose_mesh_variance");
}

// Constructor
KimeraPgmoInterface::KimeraPgmoInterface(const KimeraPgmoConfig& config)
    : config_(config::checkValid(config)),
      pgo_(config.optimizer.create()),
      full_mesh_updated_(false),
      deformation_graph_(new DeformationGraph(config_.mode == RunMode::MESH_ONLY)),
      num_loop_closures_(0) {
  pgo_->setLogPath(config.log_path);
}

bool KimeraPgmoInterface::loadGraphAndMesh(size_t robot_id,
                                           const std::string& ply_path,
                                           const std::string& dgrf_path,
                                           pcl::PolygonMesh::Ptr optimized_mesh,
                                           std::vector<Timestamp>* mesh_vertex_stamps,
                                           bool do_optimize) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  kimera_pgmo::ReadMeshWithStampsFromPly(ply_path, mesh, mesh_vertex_stamps);

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
  return deformation_graph_->getTrajectory(robot_prefix);
}

std::vector<Timestamp> KimeraPgmoInterface::getRobotTimestamps(size_t robot_id) const {
  std::vector<Timestamp> stamps;
  const char& robot_prefix = robot_id_to_prefix.at(robot_id);
  // TODO(yun) remove call to get optimized trajectory
  Path optimized_traj = deformation_graph_->getTrajectory(robot_prefix);
  for (size_t i = 0; i < optimized_traj.size(); i++) {
    stamps.push_back(keyed_stamps_.at(gtsam::Symbol(robot_prefix, i)));
  }

  return stamps;
}

const gtsam::NonlinearFactorGraph& KimeraPgmoInterface::getDeformationGraphFactors()
    const {
  return *deformation_graph_->getFactors();
}

const gtsam::Values& KimeraPgmoInterface::getDeformationGraphValues() const {
  return *deformation_graph_->getValues();
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

void KimeraPgmoInterface::resetDeformationGraph() {
  deformation_graph_.reset(new DeformationGraph);
}

void KimeraPgmoInterface::loadDeformationGraphFromFile(const std::string& input) {
  deformation_graph_->load(input);
  num_loop_closures_ = deformation_graph_->getNumLoopclosures();
}

void KimeraPgmoInterface::loadDeformationGraphFromFile(const std::string& input,
                                                       size_t robot_id,
                                                       bool include_priors) {
  deformation_graph_->load(input, true, true, robot_id, include_priors);
  num_loop_closures_ = deformation_graph_->getNumLoopclosures();
}

ProcessPoseGraphStatus KimeraPgmoInterface::processIncrementalPoseGraph(
    const PoseGraph& msg,
    const std::vector<size_t>& new_mesh_indices,
    const std::vector<Timestamp>& new_mesh_index_stamps,
    Path& initial_trajectory,
    std::vector<Timestamp>& node_timestamps) {
  std::map<Timestamp, gtsam::Key> stamped_nodes;
  std::vector<gtsam::Pose3> odom_measurements;

  // if first node initialize
  //// Note that we assume for all node ids that the keys start with 0
  if (!msg.nodes.empty() && msg.nodes[0].key == 0 && initial_trajectory.empty()) {
    const auto init_node = msg.nodes[0];
    size_t robot_id = init_node.robot_id;

    const gtsam::Symbol key_symb(GetRobotPrefix(robot_id), 0);
    const gtsam::Pose3 init_pose(init_node.pose.matrix());
    if (config_.mode != RunMode::MESH_ONLY) {
      deformation_graph_->processNewNode(
          key_symb, init_pose, config_.b_add_initial_prior, config_.prior_variance);
    }

    keyed_stamps_.insert({key_symb, init_node.stamp_ns});
    stamped_nodes[init_node.stamp_ns] = key_symb;

    // Add to trajectory and timestamp map
    initial_trajectory.push_back(init_pose);
    node_timestamps.push_back(init_node.stamp_ns);

    // Push node to queue to be connected to mesh vertices later
    SPARK_LOG(DEBUG) << "Initialized first node in pose graph.";
  }

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
        SPARK_LOG(ERROR) << "Odometry edge should not connect two nodes from different "
                            "robots.";
        return ProcessPoseGraphStatus::INVALID;
      }

      if (to_key.index() != initial_trajectory.size()) {
        if (to_key.index() > initial_trajectory.size()) {
          SPARK_LOG(ERROR) << "Missing from node "
                           << gtsam::DefaultKeyFormatter(from_key)
                           << " in odometry edge.";
          return ProcessPoseGraphStatus::MISSING;
        }
        SPARK_LOG(WARNING) << "Duplicated edge";
        continue;
      }

      // Calculate pose of new node
      const auto& new_pose = initial_trajectory.at(from_key.index()).compose(measure);
      initial_trajectory.push_back(new_pose);

      node_timestamps.push_back(pg_edge.stamp_ns);
      keyed_stamps_.insert({to_key, pg_edge.stamp_ns});

      // For mesh only also need to add the previous node to stamped_nodes for
      // connectivity
      if (config_.mode == RunMode::MESH_ONLY) {
        auto from_stamp = keyed_stamps_.at(from_key);
        if (!stamped_nodes.count(from_stamp)) {
          stamped_nodes[from_stamp] = from_key;
        }
      }

      stamped_nodes[pg_edge.stamp_ns] = to_key;
      odom_measurements.push_back(measure);

      if (config_.mode == RunMode::MESH_ONLY) {
        continue;
      }

      deformation_graph_->updatePoseGraphInitialGuess(from_key, to_key, measure);
      deformation_graph_->processNewBetween(
          from_key, to_key, measure, config_.odom_variance);

    } else if (pg_edge.type == pose_graph_tools::PoseGraphEdge::LOOPCLOSE) {
      if (!keyed_stamps_.count(from_key) || !keyed_stamps_.count(to_key)) {
        SPARK_LOG(ERROR) << "Caught loop closure between unknown nodes.";
        return ProcessPoseGraphStatus::LC_MISSING_NODES;
      }

      if (loop_closures_.count(from_key) && loop_closures_.at(from_key).count(to_key)) {
        // Loop closure already exists TODO(yun) add flag to toggle this check
        continue;
      }

      if (config_.mode == RunMode::FULL) {
        // Loop closure edge (only add if we are in full
        // optimization mode ) Add to deformation graph
        deformation_graph_->processNewBetween(
            from_key, to_key, measure, config_.lc_variance);
      } else if (config_.mode == RunMode::MESH_ONLY) {
        const auto& from_stamp = keyed_stamps_.at(from_key);
        const auto& to_stamp = keyed_stamps_.at(to_key);
        if (!addMeshMeshConnections({{from_stamp, from_key}, {to_stamp, to_key}},
                                    {measure},
                                    initial_trajectory,
                                    false)) {  // not necessarily inliers
          SPARK_LOG(WARNING)
              << "Failed to add mesh-to-mesh connections for loop closure.";
        }
        deformation_graph_->setRecalculateVertices();
      }
      auto& loop_closures_to = loop_closures_[from_key];
      loop_closures_to.insert(to_key);
      auto& loop_closures_from = loop_closures_[to_key];
      loop_closures_from.insert(from_key);
      SPARK_LOG(INFO) << "KimeraPgmo: Loop closure detected between robot "
                      << robot_from << " node " << prev_node << " and robot "
                      << robot_to << " node " << current_node << ".";
      num_loop_closures_++;
    }
  }

  if (config_.mode != RunMode::MESH_ONLY) {
    if (!addPoseMeshConnections(stamped_nodes)) {
      if (stamped_nodes.size() > 0 && new_mesh_indices.size() > 0) {
        SPARK_LOG(WARNING) << "KimeraPgmo: Partial mesh not connected to pose graph.";
      }
      return ProcessPoseGraphStatus::MESH_DISCONNECTED;
    }
  } else {
    if (!addMeshMeshConnections(stamped_nodes,
                                odom_measurements,
                                initial_trajectory,
                                true)) {  // as inliers
      if (stamped_nodes.size() > 0 && new_mesh_indices.size() > 0) {
        SPARK_LOG(WARNING) << "KimeraPgmo: Partial mesh not connected to pose graph.";
      }
      return ProcessPoseGraphStatus::MESH_DISCONNECTED;
    }
  }
  return ProcessPoseGraphStatus::SUCCESS;
}

bool KimeraPgmoInterface::findClosestMeshIndices(const size_t& robot_id,
                                                 const Timestamp& stamp,
                                                 std::vector<size_t>& indices) {
  if (!deformation_graph_->hasVertexKey(GetVertexPrefix(robot_id))) {
    return false;
  }
  const auto& mesh_stamps =
      deformation_graph_->getVertexStamps(GetVertexPrefix(robot_id));
  if (mesh_stamps.empty()) {
    return false;
  }

  // TODO(Yun): Implicit assumption here that the mesh stamps are ordered (normally are
  // since that's how processIncrementalMeshGraph works) but might need to account for
  // out-of-order timestamps
  auto lower = std::lower_bound(mesh_stamps.begin(), mesh_stamps.end(), stamp);
  size_t idx = std::distance(mesh_stamps.begin(), lower);

  size_t closest;
  if (idx == 0) {
    closest = mesh_stamps.front();
  } else if (idx == mesh_stamps.size()) {
    closest = mesh_stamps.back();
  } else {
    size_t prev = mesh_stamps[idx - 1];
    size_t curr = mesh_stamps[idx];

    closest = (stamp - prev <= curr - stamp) ? prev : curr;
  }

  // Collect other indices with same value
  for (size_t i = 0; i < mesh_stamps.size(); i++) {
    if (mesh_stamps[i] == closest) {
      indices.push_back(i);
    }
  }
  return true;
}

bool KimeraPgmoInterface::addPoseMeshConnections(
    const std::map<Timestamp, gtsam::Key>& stamped_nodes) {
  bool connection = false;
  for (const auto& [stamp, key] : stamped_nodes) {
    const auto robot_id = robot_prefix_to_id.at(gtsam::Symbol(key).chr());
    std::vector<size_t> vertex_indices;
    if (!findClosestMeshIndices(robot_id, stamp, vertex_indices)) {
      continue;
    }

    if (vertex_indices.empty()) {
      continue;
    }
    connection = true;
    deformation_graph_->processNodeValence(
        key, vertex_indices, GetVertexPrefix(robot_id), config_.pose_mesh_variance);
  }
  return connection;
}

bool KimeraPgmoInterface::addMeshMeshConnections(
    const std::map<Timestamp, gtsam::Key>& stamped_nodes,
    const std::vector<gtsam::Pose3>& measurements,
    const Path& initial_trajectory,
    bool as_inliers) {
  if (stamped_nodes.size() == 0) {
    return true;
  }

  auto first_node_it = stamped_nodes.begin();
  gtsam::Symbol prev_node = gtsam::Symbol(first_node_it->second);
  Timestamp prev_stamp = first_node_it->first;
  size_t prev_robot_id = robot_prefix_to_id.at(prev_node.chr());
  std::vector<size_t> prev_valences;
  gtsam::Pose3 prev_pose = initial_trajectory.at(prev_node.index());
  findClosestMeshIndices(prev_robot_id, prev_stamp, prev_valences);

  bool connection = false;
  size_t measurement_idx = 0;
  // Iterate through current nodes
  for (auto node_it = std::next(first_node_it); node_it != stamped_nodes.end();
       ++node_it) {
    auto [curr_stamp, curr_key] = *node_it;
    auto curr_node = gtsam::Symbol(curr_key);
    auto curr_robot_id = robot_prefix_to_id.at(curr_node.chr());
    auto curr_pose = initial_trajectory.at(curr_node.index());
    std::vector<size_t> curr_valences;
    findClosestMeshIndices(curr_robot_id, curr_stamp, curr_valences);

    if (!curr_valences.empty() && !prev_valences.empty()) {
      connection = true;
      deformation_graph_->processBetweenAsMeshConnections(
          prev_pose,
          prev_valences,
          curr_pose,
          curr_valences,
          measurements.at(measurement_idx),
          GetVertexPrefix(prev_robot_id),
          GetVertexPrefix(curr_robot_id),
          config_.mesh_edge_variance,
          false,  // Not temp
          as_inliers);
    }

    prev_node = std::move(curr_node);
    prev_stamp = curr_stamp;
    prev_robot_id = curr_robot_id;
    prev_pose = std::move(curr_pose);
    prev_valences = std::move(curr_valences);
    measurement_idx++;
  }
  return connection;
}

ProcessMeshGraphStatus KimeraPgmoInterface::processIncrementalMeshGraph(
    const PoseGraph& mesh_graph,
    const std::vector<Timestamp>& node_timestamps,
    std::vector<size_t>& new_mesh_indices,
    std::vector<Timestamp>& new_mesh_index_stamps) {
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
  deformation_graph_->processNewMeshEdgesAndNodes(new_mesh_edges,
                                                  new_mesh_nodes,
                                                  new_mesh_node_stamps,
                                                  &new_mesh_indices,
                                                  &new_mesh_index_stamps,
                                                  config_.mesh_edge_variance);
  assert(new_mesh_indices.size() == new_mesh_index_stamps.size());

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

  deformation_graph_->processNodeMeasurements(node_estimates, config_.prior_variance);
}

OptimizeStats KimeraPgmoInterface::optimize() {
  gtsam::NonlinearFactorGraph factors, temp_factors;
  gtsam::Values initial, temp_initial;
  std::set<size_t> known_inliers, temp_known_inliers;
  {
    auto lock = deformation_graph_->acquireLock();
    optimizeStartup();
    factors = deformation_graph_->getFactorsCopy();
    initial = deformation_graph_->getValuesCopy();
    temp_factors = deformation_graph_->getTempFactorsCopy();
    temp_initial = deformation_graph_->getTempValuesCopy();
    known_inliers = deformation_graph_->getKnownInlierSetCopy();
    temp_known_inliers = deformation_graph_->getTempKnownInlierSetCopy();
  }

  const auto start = std::chrono::system_clock::now();
  pgo_->update(
      factors, initial, known_inliers, temp_factors, temp_initial, temp_known_inliers);
  const auto end = std::chrono::system_clock::now();

  auto estimates = pgo_->getEstimates();
  auto temp_estimates = pgo_->getTempEstimates();
  auto inlier_weights = pgo_->getInlierWeights();
  auto temp_inlier_weights = pgo_->getTempInlierWeights();

  size_t index = 0;
  OptimizeStats stats;
  stats.total_factors = factors.size() + temp_factors.size();
  stats.total_values = estimates.size() + temp_estimates.size();
  for (const auto& factor : factors) {
    const auto derived = dynamic_cast<const BetweenFactorT*>(factor.get());
    if (!derived) {
      continue;
    }

    const gtsam::Symbol k1(derived->keys().front());
    const gtsam::Symbol k2(derived->keys().back());
    const int64_t k1_index = k1.index();
    const int64_t k2_index = k2.index();
    if (std::abs(k1_index - k2_index) <= 1) {
      continue;  // odometry
    }

    bool interrobot = k1.chr() != k2.chr();
    bool inlier = true;
    if (inlier_weights.size() < index && inlier_weights[index] < 0.5) {
      inlier = false;
    }

    ++index;
    stats.total_loop_closures += 1;
    stats.inlier_loop_closures += (inlier ? 1 : 0);
    if (interrobot) {
      stats.total_interrobot_loop_closures += 1;
      stats.inlier_interrobot_loop_closures += (inlier ? 1 : 0);
    }
  }

  stats.elapsed_s =
      std::chrono::duration_cast<std::chrono::duration<double>>(end - start).count();

  {
    auto lock = deformation_graph_->acquireLock();
    deformation_graph_->updateValues(estimates);
    deformation_graph_->updateTempValues(temp_estimates);
    deformation_graph_->updateInlierWeights(inlier_weights);
    deformation_graph_->updateTempInlierWeights(temp_inlier_weights);
    optimizeCleanup(estimates);
  }

  return stats;
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
    if (do_optimize) {
      optimize();
    }

    optimized_mesh = deformation_graph_->deformMesh(input_mesh,
                                                    mesh_vertex_stamps,
                                                    mesh_vertex_graph_inds,
                                                    GetVertexPrefix(robot_id),
                                                    config_.num_interp_pts,
                                                    config_.interp_horizon);
  } catch (const std::out_of_range& e) {
    SPARK_LOG(ERROR) << "Failed to deform mesh. Out of range error.";
    return false;
  }

  full_mesh_updated_ = true;
  return true;
}

bool KimeraPgmoInterface::saveMesh(const pcl::PolygonMesh& mesh,
                                   const std::string& ply_name) const {
  // Save mesh
  WriteMeshToPly(ply_name, mesh);
  SPARK_LOG(INFO) << "KimeraPgmo: Saved mesh to file.";
  return true;
}

bool KimeraPgmoInterface::saveTrajectory(const Path& trajectory,
                                         const std::vector<Timestamp>& timestamps,
                                         const std::string& csv_file) const {
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

bool KimeraPgmoInterface::saveDeformationGraph(const std::string& dgrf_name) const {
  // Save mesh
  deformation_graph_->save(dgrf_name);
  SPARK_LOG(INFO) << "KimeraPgmo: Saved deformation graph to file.";
  return true;
}

void KimeraPgmoInterface::setVerboseFlag(bool verbose) {
  verbose_ = verbose;
  if (deformation_graph_) {
    deformation_graph_->setVerboseFlag(verbose);
  }
}

}  // namespace kimera_pgmo
