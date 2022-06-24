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

  /*! \brief Get the optimized trajectory of a robot
   * - robot_id: id of the robot referred to in query
   */
  Path getOptimizedTrajectory(const size_t& robot_id) const;

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
                        pcl::PolygonMesh::Ptr optimized_mesh,
                        std::vector<ros::Time>* mesh_vertex_stamps,
                        bool do_optimize);

 protected:
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
  void processIncrementalPoseGraph(
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
  void processIncrementalMeshGraph(
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

  /*! \brief Get the consistency factors as pose graph edges
   * - robot_id: the id of the robot in question
   * - pg_mesh_msg: pointer to the factors and initial values
   * - vertex_index_offset start index for vertices from this index (default 0)
   */
  bool getConsistencyFactors(const size_t& robot_id,
                             pose_graph_tools::PoseGraph* pg_mesh_msg,
                             const size_t& vertex_index_offset = 0) const;

  void inline insertDpgmoValues(const gtsam::Key& key,
                                const gtsam::Pose3& pose) {
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
  void visualizeDeformationGraphMeshEdges(
      const ros::Publisher* mesh_mesh_pub,
      const ros::Publisher* pose_mesh_pub) const;

  inline void setVerboseFlag(bool verbose) {
    verbose_ = verbose;
    if (deformation_graph_) {
      deformation_graph_->setVerboseFlag(verbose);
    }
  }

 protected:
  enum class RunMode {
    FULL = 0u,  // Optimize mesh and pose graph
    MESH = 1u,  // Optimize mesh based on given optimized trajectory
    DPGMO = 2u  // DPGO does the optimization
  };
  RunMode run_mode_;
  bool use_msg_time_;  // use msg time or call back time

  bool verbose_;  // whether or not to print messages

  bool full_mesh_updated_;

  DeformationGraphPtr deformation_graph_;
  // maximum time allowed when associating node to mesh
  double embed_delta_t_;
  // time horizon when creating simplifed mesh
  double compression_time_horizon_;

  // number of control pts to consider when interpolating for full mesh (k)
  int num_interp_pts_;

  // time horizon within which a control point is valid
  double interp_horizon_;

  // Track number of loop closures
  size_t num_loop_closures_;

  // DPGMO optimized values
  gtsam::Values dpgmo_values_;

  bool b_add_initial_prior_;
  // Covariances
  double odom_variance_;
  double lc_variance_;
  double prior_variance_;
  double mesh_edge_variance_;
  double pose_mesh_variance_;
};
}  // namespace kimera_pgmo
