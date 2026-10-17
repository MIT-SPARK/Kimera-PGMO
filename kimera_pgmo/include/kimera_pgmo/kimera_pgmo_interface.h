/**
 * @file   kimera_pgmo_interface.h
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#pragma once

#include <config_utilities/virtual_config.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <pose_graph_tools/pose_graph.h>

#include <map>
#include <string>

#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"
#include "kimera_pgmo/optimizer/optimizer_interface.h"

namespace kimera_pgmo {

enum class ProcessPoseGraphStatus {
  EMPTY,
  INVALID,
  MISSING,
  UNKNOWN,
  DUPLICATE,
  SUCCESS,
  LC_MISSING_NODES,
  MESH_DISCONNECTED
};

enum class ProcessMeshGraphStatus {
  EMPTY,
  INVALID,
  UNKNOWN,
  DUPLICATE,
  SUCCESS,
  WAITING
};

enum class RunMode {
  FULL = 0u,                // Optimize mesh and pose graph
  EXTERNAL_OPTIMIZER = 1u,  // Optimize mesh based on given optimized trajectory
  MESH_ONLY = 2u,           // No pose graph in deformation graph
};

struct OptimizeStats {
  size_t total_factors = 0;
  size_t total_values = 0;
  size_t total_loop_closures = 0;
  size_t inlier_loop_closures = 0;
  size_t total_interrobot_loop_closures = 0;
  size_t inlier_interrobot_loop_closures = 0;
  double elapsed_s = 0.0;
};

std::ostream& operator<<(std::ostream& out, const OptimizeStats& stats);

struct KimeraPgmoConfig {
  KimeraPgmoConfig() = default;

  // pgmo behavior
  RunMode mode = RunMode::FULL;
  int num_interp_pts = 3;
  double interp_horizon = 5.0;
  bool b_add_initial_prior = true;
  // covariances
  double odom_variance = -1.0;
  double lc_variance = -1.0;
  double prior_variance = -1.0;
  double mesh_edge_variance = -1.0;
  double pose_mesh_variance = -1.0;
  // logging
  std::string log_path;

  // optimizer
  config::VirtualConfig<Optimizer> optimizer{KimeraRpgoOptimizer::Config()};
};

void declare_config(KimeraPgmoConfig& config);

class KimeraPgmoInterface {
  friend class KimeraPgmoInterfaceTest;

 public:
  KimeraPgmoInterface(const KimeraPgmoConfig& config);

  ~KimeraPgmoInterface() = default;

  //! Load deformation graph
  void loadGraph(const std::string& input,
                 std::optional<size_t> robot_id = std::nullopt);

  //! Load deformation graph, mesh and potentially optimize
  bool loadGraphAndMesh(size_t robot_id,
                        const std::string& ply_path,
                        const std::string& dgrf_path,
                        bool do_optimize);

  //! Get the optimized trajectory of a robot
  std::vector<gtsam::Pose3> getOptimizedTrajectory(size_t robot_id) const;

  //! Get the timestamps of the robot trajectory
  std::vector<Timestamp> getRobotTimestamps(size_t robot_id) const;

  //! Get the factors of the underlying deformation graph
  const gtsam::NonlinearFactorGraph& getDeformationGraphFactors() const;

  //! Get the estimates of the underlying deformation graph
  const gtsam::Values& getDeformationGraphValues() const;

  //! Pointer to deformation graph
  DeformationGraph::Ptr getDeformationGraphPtr() const;

  //! Reset deformation graph
  void resetDeformationGraph();

 protected:
  ProcessPoseGraphStatus processIncrementalPoseGraph(
      const pose_graph_tools::PoseGraph& pose_graph,
      const std::vector<size_t>& new_mesh_indices,
      const std::vector<Timestamp>& new_mesh_index_stamps,
      std::vector<gtsam::Pose3>& initial_trajectory,
      std::vector<Timestamp>& node_timestamps);

  ProcessMeshGraphStatus processIncrementalMeshGraph(
      const pose_graph_tools::PoseGraph& mesh_graph_msg,
      const std::vector<Timestamp>& node_timestamps,
      std::vector<size_t>& new_mesh_indices,
      std::vector<Timestamp>& new_mesh_index_stamps);

  void processOptimizedPath(const std::vector<gtsam::Pose3>& path, size_t robot_id = 0);

  OptimizeStats optimize();

  virtual void optimizeStartup();

  virtual void optimizeCleanup(const gtsam::Values& results);

  bool deformFullMesh(size_t robot_id, bool do_optimize);

  bool addPoseMeshConnections(const std::map<Timestamp, gtsam::Key>& stamped_nodes);

  bool addMeshMeshConnections(const std::map<Timestamp, gtsam::Key>& nodes,
                              const std::vector<gtsam::Pose3>& measurements,
                              const std::vector<gtsam::Pose3>& initial_trajectory,
                              bool as_inliers);

  bool saveMesh(const std::string& filepath) const;

  bool saveTrajectory(const std::string& filepath) const;

  bool saveDeformationGraph(const std::string& filepath) const;

 protected:
  KimeraPgmoConfig config_;
  Optimizer::Ptr pgo_;

  DeformationGraph::Ptr deformation_graph_;
  double compression_time_horizon_;

  std::vector<Timestamp> timestamps_;
  std::vector<gtsam::Pose3> optimized_path_;

  std::vector<traits::Face> faces_;
  std::vector<traits::Vertex> original_vertices_;
  std::vector<traits::Vertex> optimized_vertices_;

  size_t num_loop_closures_;
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>> loop_closures_;

  std::unordered_map<gtsam::Key, Timestamp> keyed_stamps_;
};

}  // namespace kimera_pgmo
