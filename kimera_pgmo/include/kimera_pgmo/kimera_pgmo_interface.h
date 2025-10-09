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
#include <queue>
#include <string>

#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"
#include "kimera_pgmo/optimizer/optimizer_interface.h"
#include "kimera_pgmo/utils/common_functions.h"
namespace kimera_pgmo {

using Path = std::vector<gtsam::Pose3>;
using PathPtr = std::shared_ptr<Path>;

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

  /*! \brief Get the factors of the underlying deformation graph
   */
  const gtsam::NonlinearFactorGraph& getDeformationGraphFactors() const;

  /*! \brief Get the estimates of the underlying deformation graph
   */
  const gtsam::Values& getDeformationGraphValues() const;

  /*! \brief Ptr to deformation graph
   */
  DeformationGraphPtr getDeformationGraphPtr() const;

  /*! \brief get whether the mesh has been updated
   */
  bool wasFullMeshUpdated(bool clear_flag = true);

  /*! \brief Reset deformation graph
   */
  void resetDeformationGraph();

  /*! \brief Load deformation graph
   * - input: dgrf file (deformation graph file)
   */
  void loadDeformationGraphFromFile(const std::string& input);

  /*! \brief Load deformation graph and assign specific robot id
   * - input: dgrf file (deformation graph file)
   * - robot_id: robot id
   */
  void loadDeformationGraphFromFile(const std::string& input,
                                    size_t robot_id,
                                    bool include_priors = true);

 protected:
  /*! \brief Recieves latest edges in the pose graph and add to deformation
   * graph. Also updates the initial trajectory, the node connection queue, and
   * the node timestamps
   * - msg: incremental pose graph msg
   * - new_mesh_indices: new mesh indices to connect pose-mesh connections
   * - new_mesh_index_stamps: timestamps corresponding to new mesh indices
   * - initial_trajectory: vector storing the initial poses of all the odometric
   * nodes
   * - node_timestamps: vector of the timestamps of each odometric node
   */
  ProcessPoseGraphStatus processIncrementalPoseGraph(
      const pose_graph_tools::PoseGraph& pose_graph,
      const std::vector<size_t>& new_mesh_indices,
      const std::vector<Timestamp>& new_mesh_index_stamps,
      Path& initial_trajectory,
      std::vector<Timestamp>& node_timestamps);

  /*! \brief Process the mesh graph that consists of the new mesh edges and mesh
   * nodes to be added to the deformation graph
   * - mesh_msg: partial mesh in mesh_msgs TriangleMeshStamped format
   * - node_timestamps: vector of the timestamps of each odometric node
   * - unconnected_nodes: odometric nodes not yet connected to the mesh and
   * still within the embed time window
   */
  ProcessMeshGraphStatus processIncrementalMeshGraph(
      const pose_graph_tools::PoseGraph& mesh_graph_msg,
      const std::vector<Timestamp>& node_timestamps,
      std::vector<size_t>& new_mesh_indices,
      std::vector<Timestamp>& new_mesh_index_stamps);

  /*! \brief Given an optimized trajectory, adjust the mesh. The path should
   * correspond to the nodes of the pose graph received in the
   * incrementalPoseGraphCallback. Note that this is currently only supported in
   * the single robot pose graph case.
   *  - path: optimized path
   * - robot_id: id of the robot this path corresponds to
   */
  void processOptimizedPath(const Path& path, size_t robot_id = 0);

  /*! \brief Optimize the deformation graph
   */
  OptimizeStats optimize();

  /*! \brief Optimize additional startup implementation (like adding temp factors)
   */
  virtual void optimizeStartup() {}

  /*! \brief Optimize additional spindown implementation
   */
  virtual void optimizeCleanup(const gtsam::Values& results) {}

  /*! \brief Optimize the full mesh (and pose graph) using the deformation graph
   * then publish the deformed mesh
   *  - mesh_msg: the full unoptimized mesh in KimeraPgmoMesh format
   * format
   * - optimized_mesh: ptr to optimized (deformed) mesh
   * - do_optimize: call optimize. Optimize before deforming mesh.
   */
  bool optimizeFullMesh(size_t robot_id,
                        const pcl::PolygonMesh& input_mesh,
                        const std::vector<Timestamp>& mesh_vertex_stamps,
                        const std::vector<int> mesh_vertex_graph_inds,
                        pcl::PolygonMesh& optimized_mesh,
                        bool do_optimize);

  /*! \brief Find connections to (deformation graph) mesh vertices in deformation graph
   * based on timestamp
   *  - robot_id: robot id for vertex stamp lookup
   *  - stamp: timestamp of ndoe
   *  - indices: vertex indices
   */
  bool findClosestMeshIndices(const size_t& robot_id,
                              const Timestamp& stamp,
                              std::vector<size_t>& indices);

  /*! \brief Connect poses to mesh vertices in deformation graph. Assumes
   * updatePoseMeshConnections already called and the connections already updated in
   * node_valences_
   *  - stamped_nodes: unconnected poses keyed by timestamp
   */
  bool addPoseMeshConnections(const std::map<Timestamp, gtsam::Key>& stamped_nodes);

  /*! \brief Connect mesh vertices to mesh vertices in deformation graph based on
   * received odometry. Assumes updatePoseMeshConnections already called and the
   * connections already updated in node_valences_
   *  - stamped_nodes: new poses keyed by timestamp
   *  - measurements: pose measurements
   *  - initial_trajectory: initial pose trajectory
   *  - as_inliers: mark as inliers
   */
  bool addMeshMeshConnections(const std::map<Timestamp, gtsam::Key>& nodes,
                              const std::vector<gtsam::Pose3>& measurements,
                              const Path& initial_trajectory,
                              bool as_inliers);

  /*! \brief Saves mesh as a ply file.
   * - mesh: mesh to save
   * - ply_name: name of the ply file output
   */
  bool saveMesh(const pcl::PolygonMesh& mesh, const std::string& ply_name) const;

  /*! \brief Saves trajectory to csv files.
   * - trajectory: trajectory to save
   * - csv_file: name of the csv file to save to
   */
  bool saveTrajectory(const Path& trajectory,
                      const std::vector<Timestamp>& timestamps,
                      const std::string& csv_file) const;

  /*! \brief Saves deformation graph to file.
   * - dgrf_file: name of the file to write to
   */
  bool saveDeformationGraph(const std::string& dgrf_name) const;

  void setVerboseFlag(bool verbose);

 protected:
  bool verbose_;  // whether or not to print messages
  KimeraPgmoConfig config_;
  Optimizer::Ptr pgo_;

  bool full_mesh_updated_;

  DeformationGraphPtr deformation_graph_;
  // time horizon when creating simplifed mesh
  double compression_time_horizon_;

  // Track number of loop closures
  size_t num_loop_closures_;
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>> loop_closures_;

  // Timestamp mapping
  std::unordered_map<gtsam::Key, Timestamp> keyed_stamps_;
};

}  // namespace kimera_pgmo
