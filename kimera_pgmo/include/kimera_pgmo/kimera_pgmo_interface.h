/**
 * @file   kimera_pgmo_interface.h
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <pcl/PolygonMesh.h>
#include <pose_graph_tools/pose_graph.h>

#include <map>
#include <queue>
#include <string>

#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/sparse_keyframe.h"
#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

using Path = std::vector<gtsam::Pose3>;
using PathPtr = std::shared_ptr<Path>;

enum class RunMode {
  FULL = 0u,  // Optimize mesh and pose graph
  MESH = 1u,  // Optimize mesh based on given optimized trajectory
  DPGMO = 2u  // DPGO does the optimization
};

struct KimeraPgmoConfig {
  KimeraPgmoConfig() = default;

  KimeraRPGO::RobustSolverParams getRobustSolverParams() const;

  // pgmo behavior
  RunMode mode;
  double embed_delta_t;
  int num_interp_pts;
  double interp_horizon;
  bool b_add_initial_prior;
  // covariances
  double odom_variance;
  double lc_variance;
  double prior_variance;
  double mesh_edge_variance;
  double pose_mesh_variance;
  // pcm thresholds
  double odom_trans_threshold;
  double odom_rot_threshold;
  double pcm_trans_threshold;
  double pcm_rot_threshold;
  // gnc configuration
  double gnc_alpha;
  int gnc_max_it = 100;
  double gnc_mu_step = 1.4;
  double gnc_cost_tol = 1.0e-5;
  double gnc_weight_tol = 1.0e-4;
  bool lm_diagonal_damping = true;
  bool gnc_fix_prev_inliers = false;
  // sparsification
  bool b_enable_sparsify = false;
  double trans_sparse_dist = 1.0;
  double rot_sparse_dist = 1.2;
  // logging
  std::string log_path = "";
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
  KimeraPgmoInterface();

  ~KimeraPgmoInterface() = default;

  /*! \brief Sets the config and intializes information if config valid
   */
  virtual bool initialize(const KimeraPgmoConfig& config);

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
                        const std::string& sparse_mapping_file_path,
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
  gtsam::NonlinearFactorGraph getDeformationGraphFactors() const;

  /*! \brief Get the estimates of the underlying deformation graph
   */
  gtsam::Values getDeformationGraphValues() const;

  /*! \brief Ptr to deformation graph
   */
  DeformationGraphPtr getDeformationGraphPtr() const;

  /*! \brief get whether the mesh has been updated
   */
  bool wasFullMeshUpdated(bool clear_flag = true);

  /*! \brief force deformation graph to optimize
   */
  void forceOptimize();

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
  void loadDeformationGraphFromFile(const std::string& input, size_t robot_id);

 protected:
  bool initializeFromConfig();

  /*! \brief Recieves latest edges in the pose graph and add to deformation
   * graph. Also updates the initial trajectory, the node connection queue, and
   * the node timestamps
   * - msg: incremental pose graph msg
   * - initial_trajectory: vector storing the initial poses of all the odometric
   * nodes
   * - unconnected_nodes: nodes not yet connected to mesh vertices and still
   * within the embedded time window
   * - node_timestamps: vector of the timestamps of each odometric node
   */
  ProcessPoseGraphStatus processIncrementalPoseGraph(
      const pose_graph_tools::PoseGraph& pose_graph,
      Path& initial_trajectory,
      std::vector<Timestamp>& node_timestamps,
      std::queue<size_t>& unconnected_nodes);

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
      std::queue<size_t>& unconnected_nodes);

  /*! \brief Given an optimized trajectory, adjust the mesh. The path should
   * correspond to the nodes of the pose graph received in the
   * incrementalPoseGraphCallback. Note that this is currently only supported in
   * the single robot pose graph case.
   *  - path: optimized path
   * - robot_id: id of the robot this path corresponds to
   */
  void processOptimizedPath(const Path& path, size_t robot_id = 0);

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

  /*! \brief Saves mesh as a ply file.
   * - mesh: mesh to save
   * - ply_name: name of the ply file output
   */
  bool saveMesh(const pcl::PolygonMesh& mesh, const std::string& ply_name);

  /*! \brief Saves trajectory to csv files.
   * - trajectory: trajectory to save
   * - csv_file: name of the csv file to save to
   */
  bool saveTrajectory(const Path& trajectory,
                      const std::vector<Timestamp>& timestamps,
                      const std::string& csv_file);

  /*! \brief Saves deformation graph to file.
   * - dgrf_file: name of the file to write to
   */
  bool saveDeformationGraph(const std::string& dgrf_name);

  /*! \brief Saves the full_to_sparse_frames mapping
   * - output_path: name of the file to write to
   */
  bool savePoseGraphSparseMapping(const std::string& output_path);

  /*! \brief Loads the full_to_sparse_frames mapping
   * - input_path: name of the file to read from
   */
  bool loadPoseGraphSparseMapping(const std::string& input_path);

  /*! \brief Get the consistency factors as pose graph edges
   * - robot_id: the id of the robot in question
   * - pg_mesh_msg: pointer to the factors and initial values
   * - vertex_index_offset start index for vertices from this index (default 0)
   */
  bool getConsistencyFactors(size_t robot_id,
                             pose_graph_tools::PoseGraph& pg_mesh_msg,
                             size_t vertex_index_offset = 0) const;

  void insertDpgmoValues(const gtsam::Key& key, const gtsam::Pose3& pose);

  /*! \brief Get the DPGMO optimized values
   */
  gtsam::Values getDpgmoValues() const;

  void setVerboseFlag(bool verbose);

 protected:
  bool verbose_;  // whether or not to print messages
  KimeraPgmoConfig config_;

  bool full_mesh_updated_;

  DeformationGraphPtr deformation_graph_;
  // time horizon when creating simplifed mesh
  double compression_time_horizon_;

  // Track number of loop closures
  size_t num_loop_closures_;

  // DPGMO optimized values
  gtsam::Values dpgmo_values_;

  // Sparse key frame mapping
  std::unordered_map<gtsam::Key, gtsam::Key> full_sparse_frame_map_;
  std::unordered_map<gtsam::Key, SparseKeyframe> sparse_frames_;
  std::unordered_map<gtsam::Key, std::set<gtsam::Key>> loop_closures_;

  // Timestamp mapping
  std::unordered_map<gtsam::Key, Timestamp> keyed_stamps_;
};

}  // namespace kimera_pgmo
