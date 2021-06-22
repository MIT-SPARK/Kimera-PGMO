/**
 * @file   KimeraPgmoInterface.h
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#pragma once

#include <map>
#include <queue>
#include <string>

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
#include "kimera_pgmo/TriangleMeshIdStamped.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {
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
  bool publishPath(const std::vector<gtsam::Pose3>& path,
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
      std::vector<gtsam::Pose3>* initial_trajectory,
      std::queue<size_t>* unconnected_nodes,
      std::vector<ros::Time>* node_timestamps);

  /*! \brief Optimize the full mesh (and pose graph) using the deformation graph
   * then publish the deformed mesh
   *  - mesh_msg: the full unoptimized mesh in mesh_msgs TriangleMeshStamped
   * format
   * - publisher: associated publisher
   * returns the optimized mesh
   */
  bool optimizeFullMesh(const kimera_pgmo::TriangleMeshIdStamped& mesh_msg,
                        pcl::PolygonMesh* optimized_mesh);

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

  /*! \brief Get the optimized trajectory of a robot
   * - robot_id: id of the robot referred to in query
   */
  std::vector<gtsam::Pose3> getOptimizedTrajectory(
      const size_t& robot_id) const;

  /*! \brief Saves mesh as a ply file. Triggers through a rosservice call
   * and saves to file [output_prefix_][id].ply
   * - mesh: mesh to save
   * - ply_name: name of the ply file output
   */
  bool saveMesh(const pcl::PolygonMesh& mesh, const std::string& ply_name);

  /*! \brief Saves trajectory to csv files. Triggers
   * through a rosservice call and saves to file [output_prefix_][id].csv
   * - trajectory: trajectory to save
   * - csv_file: name of the csv file to save to
   */
  bool saveTrajectory(const std::vector<gtsam::Pose3>& trajectory,
                      const std::vector<ros::Time>& timestamps,
                      const std::string& csv_file);

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

 protected:
  enum class RunMode {
    FULL = 0u,  // Optimize mesh and pose graph
    MESH = 1u,  // Optimize mesh based on given optimized trajectory
    DPGMO = 2u  // DPGO does the optimization
  };
  RunMode run_mode_;
  bool use_msg_time_;  // use msg time or call back time

  DeformationGraph deformation_graph_;
  // maximum time allowed when associating node to mesh
  double embed_delta_t_;
  // time horizon when creating simplifed mesh
  double compression_time_horizon_;

  // Track number of loop closures
  size_t num_loop_closures_;

  // DPGMO optimized values
  gtsam::Values dpgmo_values_;
};
}  // namespace kimera_pgmo
