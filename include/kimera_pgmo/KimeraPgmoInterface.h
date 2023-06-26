/**
 * @file   KimeraPgmoInterface.h
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#pragma once

#include <map>
#include <queue>
#include <string>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/ros.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>
#include <tf2_ros/transform_broadcaster.h>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>

#include "kimera_pgmo/AbsolutePoseStamped.h"
#include "kimera_pgmo/DeformationGraph.h"
#include "kimera_pgmo/KimeraPgmoMesh.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {

typedef std::vector<gtsam::Pose3> Path;
typedef std::shared_ptr<Path> PathPtr;

enum class RunMode {
  FULL = 0u,  // Optimize mesh and pose graph
  MESH = 1u,  // Optimize mesh based on given optimized trajectory
  DPGMO = 2u  // DPGO does the optimization
};

struct KimeraPgmoConfig {
  KimeraPgmoConfig() = default;

  bool load(const ros::NodeHandle& nh);

  KimeraRPGO::RobustSolverParams getRobustSolverParams() const;

  // config state (whether or not required params have been parsed)
  bool valid = false;

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

struct SparseKeyframe {
  gtsam::Key key;
  std::map<gtsam::Key, gtsam::Pose3> keyed_transforms;
  bool active = true;
  gtsam::Pose3 current_transform;
  std::vector<pose_graph_tools::PoseGraphEdge> edges;

  void initialize(const gtsam::Key& sparse_key,
                  const size_t& robot_id,
                  const size_t& pose_id);
  bool addNewEdge(const pose_graph_tools::PoseGraphEdge& new_edge,
                  double trans_threshold,
                  double rot_threshold);
};

class KimeraPgmoInterface {
  friend class KimeraPgmoInterfaceTest;

 public:
  /*! \brief Constructor for Kimera Pgmo class. Which subscribes to the
   * incremental mesh and pose graph to create the deformation graph and also
   * the full mesh to perform distortions and publish the optimzed distored mesh
   * and trajectory
   */
  KimeraPgmoInterface();
  ~KimeraPgmoInterface();

  /*! \brief Initializes callbacks and publishers, and also parse the parameters
   *  - n: ROS node handle.
   */
  virtual bool initialize(const ros::NodeHandle& n) = 0;

  /*! \brief Get the factors of the underlying deformation graph
   */
  inline gtsam::NonlinearFactorGraph getDeformationGraphFactors() const {
    return deformation_graph_->getGtsamFactors();
  }

  /*! \brief Get the estimates of the underlying deformation graph
   */
  inline gtsam::Values getDeformationGraphValues() const {
    return deformation_graph_->getGtsamValues();
  }

  /*! \brief Ptr to deformation graph
   */
  inline DeformationGraphPtr getDeformationGraphPtr() const {
    return deformation_graph_;
  }

  /*! \brief get whether the mesh has been updated
   */
  inline bool wasFullMeshUpdated(bool clear_flag = true) {
    bool to_return = full_mesh_updated_;
    if (clear_flag) {
      full_mesh_updated_ = false;
    }
    return to_return;
  }

  /*! \brief force deformation graph to optimize
   */
  inline void forceOptimize() { return deformation_graph_->optimize(); }

  /*! \brief Get the optimized trajectory of a robot
   * - robot_id: id of the robot referred to in query
   */
  Path getOptimizedTrajectory(const size_t& robot_id) const;

  /*! \brief Get the timestamps of the robot trajectory
   * - robot_id: id of the robot referred to in query
   */
  std::vector<ros::Time> getRobotTimestamps(const size_t& robot_id) const;

  /*! \brief Reset deformation graph
   */
  void resetDeformationGraph() {
    KimeraRPGO::RobustSolverParams pgo_params = deformation_graph_->getParams();
    deformation_graph_.reset(new DeformationGraph);
    deformation_graph_->initialize(pgo_params);
  }

  /*! \brief Load deformation graph
   * - input: dgrf file (deformation graph file)
   */
  void loadDeformationGraphFromFile(const std::string& input) {
    deformation_graph_->load(input);
    num_loop_closures_ = deformation_graph_->getNumLoopclosures();
  }

  /*! \brief Load deformation graph and assign specific robot id
   * - input: dgrf file (deformation graph file)
   * - robot_id: robot id
   */
  void loadDeformationGraphFromFile(const std::string& input, const size_t& robot_id) {
    deformation_graph_->load(input, true, true, robot_id);
    num_loop_closures_ = deformation_graph_->getNumLoopclosures();
  }

  /*! \brief Load deformation graph and mesh from file
   * - robot_id: robot id
   * - ply_path: ply file storing mesh
   * - dgrf_path: dgrf file storing deformation graph
   * - optimized_mesh: ptr to optimized mesh (to be returned)
   * - do_optimize: toggle optimization
   */
  bool loadGraphAndMesh(const size_t& robot_id,
                        const std::string& ply_path,
                        const std::string& dgrf_path,
                        const std::string& sparse_mapping_file_path,
                        pcl::PolygonMesh::Ptr optimized_mesh,
                        std::vector<ros::Time>* mesh_vertex_stamps,
                        bool do_optimize);

 protected:
  bool initializeFromConfig();

  /*! \brief Load the parameters required by this class through ROS
   *  - n: ROS node handle
   */
  virtual bool loadParameters(const ros::NodeHandle& n);

  /*! \brief Creates the ROS publishers used
   *  - n: ROS node handle
   */
  virtual bool createPublishers(const ros::NodeHandle& n) = 0;

  /*! \brief Starts the callbacks in this class
   *  - n: ROS node handle
   */
  virtual bool registerCallbacks(const ros::NodeHandle& n) = 0;

  /*! \brief Publish mesh
   * - mesh: mesh to publish
   * - header: header for the published message
   * - publisher: associated ros publisher
   */
  bool publishMesh(const pcl::PolygonMesh& mesh,
                   const std_msgs::Header& header,
                   const ros::Publisher* publisher) const;

  /*! \brief Publish trajectory
   * - path: path to publish
   * - header: header for published message
   * - publisher: associated publisher
   */
  bool publishPath(const Path& path,
                   const std_msgs::Header& header,
                   const ros::Publisher* publisher) const;

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
      const pose_graph_tools::PoseGraph::ConstPtr& msg,
      Path* initial_trajectory,
      std::queue<size_t>* unconnected_nodes,
      std::vector<ros::Time>* node_timestamps);

  /*! \brief Optimize the full mesh (and pose graph) using the deformation graph
   * then publish the deformed mesh
   *  - mesh_msg: the full unoptimized mesh in KimeraPgmoMesh format
   * format
   * - optimized_mesh: ptr to optimized (deformed) mesh
   * - do_optimize: call optimize. Optimize before deforming mesh.
   */
  bool optimizeFullMesh(const KimeraPgmoMesh& mesh_msg,
                        pcl::PolygonMesh::Ptr optimized_mesh,
                        std::vector<ros::Time>* mesh_vertex_stamps,
                        bool do_optimize);

  /*! \brief Process the mesh graph that consists of the new mesh edges and mesh
   * nodes to be added to the deformation graph
   * - mesh_msg: partial mesh in mesh_msgs TriangleMeshStamped format
   * - node_timestamps: vector of the timestamps of each odometric node
   * - unconnected_nodes: odometric nodes not yet connected to the mesh and
   * still within the embed time window
   */
  ProcessMeshGraphStatus processIncrementalMeshGraph(
      const pose_graph_tools::PoseGraph::ConstPtr& mesh_graph_msg,
      const std::vector<ros::Time>& node_timestamps,
      std::queue<size_t>* unconnected_nodes);

  /*! \brief Given an optimized trajectory, adjust the mesh. The path should
   * correspond to the nodes of the pose graph received in the
   * incrementalPoseGraphCallback. Note that this is currently only supported in
   * the single robot pose graph case.
   *  - path_msg: ros msg of the optimized path
   * - robot_id: id of the robot this path corresponds to
   */
  void processOptimizedPath(const nav_msgs::Path::ConstPtr& path_msg,
                            const size_t& robot_id = 0);

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
                      const std::vector<ros::Time>& timestamps,
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
  bool getConsistencyFactors(const size_t& robot_id,
                             pose_graph_tools::PoseGraph* pg_mesh_msg,
                             const size_t& vertex_index_offset = 0) const;

  void inline insertDpgmoValues(const gtsam::Key& key, const gtsam::Pose3& pose) {
    if (dpgmo_values_.exists(key)) {
      ROS_ERROR("Attempting to insert existing key to dpgmo values. ");
      return;
    }
    dpgmo_values_.insert(key, pose);
  }
  /*! \brief Get the DPGMO optimized values
   */
  gtsam::Values inline getDpgmoValues() const { return dpgmo_values_; }

  /*! \brief visualize the mesh-to-mesh edges and pose-to-mesh edges of the
  deformation graph
  - mesh_mesh_pub: publisher publishing the mesh to mesh edges
  - pose_mesh_pub: publisher publishing the pose to mesh edges
   */
  void visualizeDeformationGraphMeshEdges(const ros::Publisher* mesh_mesh_pub,
                                          const ros::Publisher* pose_mesh_pub) const;

  inline void setVerboseFlag(bool verbose) {
    verbose_ = verbose;
    if (deformation_graph_) {
      deformation_graph_->setVerboseFlag(verbose);
    }
  }

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
  std::unordered_map<gtsam::Key, ros::Time> keyed_stamps_;
};

}  // namespace kimera_pgmo
