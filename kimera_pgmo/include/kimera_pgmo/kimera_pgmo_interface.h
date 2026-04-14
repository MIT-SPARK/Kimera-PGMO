/**
 * @file   kimera_pgmo_interface.h
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#pragma once

#include <config_utilities/virtual_config.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/PolygonMesh.h>
#include <pose_graph_tools/pose_graph.h>

#include <map>
#include <string>

#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"
#include "kimera_pgmo/optimizer/optimizer_interface.h"

namespace kimera_pgmo {

using Path = std::vector<gtsam::Pose3>;
using PathPtr = std::shared_ptr<Path>;

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
  /*! \brief Constructor for Kimera Pgmo class. Which subscribes to the
   * incremental mesh and pose graph to create the deformation graph and also
   * the full mesh to perform distortions and publish the optimzed distored mesh
   * and trajectory
   */
  KimeraPgmoInterface(const KimeraPgmoConfig& config);

  ~KimeraPgmoInterface() = default;

  /*! \brief Load deformation graph and mesh from file
   * - robot_id: robot id
   * - ply_path: ply file storing mesh
   * - dgrf_path: dgrf file storing deformation graph
   * - optimized_mesh: ptr to optimized mesh (to be returned)
   * - do_optimize: toggle optimization
   */
  bool loadGraphAndMesh(size_t robot_id,
                        const std::string& ply_path,
                        const std::string& dgrf_path,
                        pcl::PolygonMesh::Ptr optimized_mesh,
                        std::vector<Timestamp>* mesh_vertex_stamps,
                        bool do_optimize);

  /*! \brief Get the optimized trajectory of a robot
   * - robot_id: id of the robot referred to in query
   */
  Path getOptimizedTrajectory(size_t robot_id) const;

  /*! \brief Get the timestamps of the robot trajectory
   * - robot_id: id of the robot referred to in query
   */
  std::vector<Timestamp> getRobotTimestamps(size_t robot_id) const;

  //! Get the factors of the underlying deformation graph
  const gtsam::NonlinearFactorGraph& getDeformationGraphFactors() const;

  //! Get the estimates of the underlying deformation graph
  const gtsam::Values& getDeformationGraphValues() const;

  //! Pointer to deformation graph
  DeformationGraph::Ptr getDeformationGraphPtr() const;

  //! Get whether the mesh has been updated
  bool wasFullMeshUpdated(bool clear_flag = true);

  //! Reset deformation graph
  void resetDeformationGraph();

  //! Load deformation graph
  void loadDeformationGraphFromFile(const std::string& input);

  /*! \brief Load deformation graph and assign specific robot id
   * - input: dgrf file (deformation graph file)
   * - robot_id: robot id
   */
  void loadDeformationGraphFromFile(const std::string& input,
                                    size_t robot_id,
                                    bool include_priors = true);

 protected:
  ProcessPoseGraphStatus processIncrementalPoseGraph(
      const pose_graph_tools::PoseGraph& pose_graph,
      const std::vector<size_t>& new_mesh_indices,
      const std::vector<Timestamp>& new_mesh_index_stamps,
      Path& initial_trajectory,
      std::vector<Timestamp>& node_timestamps);

  ProcessMeshGraphStatus processIncrementalMeshGraph(
      const pose_graph_tools::PoseGraph& mesh_graph_msg,
      const std::vector<Timestamp>& node_timestamps,
      std::vector<size_t>& new_mesh_indices,
      std::vector<Timestamp>& new_mesh_index_stamps);

  void processOptimizedPath(const Path& path, size_t robot_id = 0);

  OptimizeStats optimize();

  virtual void optimizeStartup() {}

  virtual void optimizeCleanup(const gtsam::Values& /* results */) {}

  bool optimizeFullMesh(size_t robot_id,
                        const pcl::PolygonMesh& input_mesh,
                        const std::vector<Timestamp>& mesh_vertex_stamps,
                        const std::vector<int>& mesh_vertex_graph_inds,
                        pcl::PolygonMesh& optimized_mesh,
                        bool do_optimize);

  bool findClosestMeshIndices(const size_t& robot_id,
                              const Timestamp& stamp,
                              std::vector<size_t>& indices);

  bool addPoseMeshConnections(const std::map<Timestamp, gtsam::Key>& stamped_nodes);

  bool addMeshMeshConnections(const std::map<Timestamp, gtsam::Key>& nodes,
                              const std::vector<gtsam::Pose3>& measurements,
                              const Path& initial_trajectory,
                              bool as_inliers);

  bool saveMesh(const pcl::PolygonMesh& mesh, const std::string& ply_name) const;

  bool saveTrajectory(const Path& trajectory,
                      const std::vector<Timestamp>& timestamps,
                      const std::string& csv_file) const;

  bool saveDeformationGraph(const std::string& dgrf_name) const;

 protected:
  KimeraPgmoConfig config_;
  Optimizer::Ptr pgo_;

  bool full_mesh_updated_;

  DeformationGraph::Ptr deformation_graph_;
  double compression_time_horizon_;

  size_t num_loop_closures_;
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>> loop_closures_;

  std::unordered_map<gtsam::Key, Timestamp> keyed_stamps_;
};

}  // namespace kimera_pgmo
