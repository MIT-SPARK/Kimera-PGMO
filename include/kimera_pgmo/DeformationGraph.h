/**
 * @file   DeformationGraph.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */
#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <KimeraRPGO/RobustSolver.h>

#include <geometry_msgs/Pose.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <visualization_msgs/Marker.h>

#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/CommonStructs.h"

namespace kimera_pgmo {

/*! \brief Define a factor type for edges between two mesh vertices or between a
 * mesh vertex and a pose graph node to be added to deformation graph. Inherited
 * from GTSAM NoiseModelFactor2
 */
class DeformationEdgeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  gtsam::Pose3 node1_pose;
  gtsam::Point3 node2_position;

 public:
  DeformationEdgeFactor(gtsam::Key node1_key,
                        gtsam::Key node2_key,
                        const gtsam::Pose3& node1_pose,
                        const gtsam::Point3& node2_point,
                        gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model,
                                                             node1_key,
                                                             node2_key),
        node1_pose(node1_pose),
        node2_position(node2_point) {}
  ~DeformationEdgeFactor() {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1,
                              const gtsam::Pose3& p2,
                              boost::optional<gtsam::Matrix&> H1 = boost::none,
                              boost::optional<gtsam::Matrix&> H2 = boost::none) const {
    // position of node 2 in frame of node 1
    gtsam::Point3 t_12 = node1_pose.rotation().inverse().rotate(
        node2_position - node1_pose.translation());

    gtsam::Matrix H_R1, H_t1, H_t2;
    gtsam::Rot3 R1 = p1.rotation();
    gtsam::Point3 t1 = p1.translation(H_t1);
    // New position of node 2 according to deformation p1 of node 1
    gtsam::Point3 t2_1 = t1 + R1.rotate(t_12, H_R1);
    gtsam::Point3 t2_2 = p2.translation(H_t2);

    // Calculate Jacobians
    Eigen::MatrixXd Jacobian_1 = Eigen::MatrixXd::Zero(3, 6);
    Jacobian_1.block<3, 3>(0, 0) = H_R1;
    Jacobian_1 = Jacobian_1 + H_t1;
    Eigen::MatrixXd Jacobian_2 = Eigen::MatrixXd::Zero(3, 6);
    Jacobian_2 = Jacobian_2 - H_t2;

    if (H1) *H1 = Jacobian_1;
    if (H2) *H2 = Jacobian_2;

    return t2_1 - t2_2;
  }

  inline gtsam::Pose3 fromPose() const { return node1_pose; }
  inline gtsam::Point3 toPoint() const { return node2_position; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new DeformationEdgeFactor(*this)));
  }
};

class DeformationGraph {
 public:
  /*! \brief Deformation graph class constructor
   */
  DeformationGraph();
  ~DeformationGraph();

  inline void setVerboseFlag(bool verbose) { verbose_ = verbose; }

  /*! \brief Initialize deformation graph along with robust solver backend.
   *  - odom_trans_threshold: translation threshold (meters of error per node)
   * for odometry check outlier rejection
   *  - odom_rot_threshold: rotation threshold (radians of error per node) for
   * odometry check outlier rejection
   *  - pcm_trans_threshold: translation threshold (meters of error per node)
   * for backend PCM outlier rejection
   *  - pcm_rot_threshold: rotation threshold (radians of error per node) for
   * backend PCM outlier rejection
   */
  bool initialize(const KimeraRPGO::RobustSolverParams& params);

  /*! \brief Fix the transform of a node corresponding to a sampled mesh vertex
   * in deformation graph. Note that all vertices has an original rotation of
   * identity.
   *  - v: Vertex (index) to impose transform on
   *  - transform: imposed transform (geometry_msgs Pose)
   *  - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   *  - variance: covariance of the prior to attach
   */
  void addMeasurement(const Vertex& v,
                      const geometry_msgs::Pose& transform,
                      const char& prefix,
                      double variance = 1e-10);

  /*! \brief Fix the transform of a node corresponding to a pose graph node
   *  - key: Key of the node to transform. Note that this is the key after
   * adding the prefix, etc gtsam::Symbol(prefix, index).key()
   *  - pose: Transform to impose (GTSAM Pose3)
   *  - variance: covariance of the prior
   */
  void addNodeMeasurement(const gtsam::Key& key,
                          const gtsam::Pose3 pose,
                          double variance = 1e-4);

  /*! \brief Fix the measurements of multiple nodes
   *  - measurements: a vector of key->pose pair of node measurements
   *  - variance: covariance of the prior factors
   */
  void addNodeMeasurements(
      const std::vector<std::pair<gtsam::Key, gtsam::Pose3> >& measurements,
      double variance = 1e-4);

  /*! \brief Initialize with new node of a trajectory
   *  - key: Key of first node in new trajectory
   *  - initial_pose: Initial measurement of first node
   *  - add_prior: boolean - add a Prior Factor or not
   *  - prior_variance: covariance of the prior
   */
  void addNewNode(const gtsam::Key& key,
                  const gtsam::Pose3& initial_pose,
                  bool add_prior,
                  double prior_variance = 1e-8);

  /*! \brief Initialize with new node of a trajectory, but keep it temporary
   *  - key: Key of first node in new trajectory
   *  - initial_pose: Initial measurement of first node
   *  - add_prior: boolean - add a Prior Factor or not
   *  - prior_variance: covariance of the prior
   */
  void addNewTempNode(const gtsam::Key& key,
                      const gtsam::Pose3& initial_pose,
                      bool add_prior,
                      double prior_variance = 1e-8);

  /*! \brief Add nodes and their valences, but keep them temporary
   *  - keys: vector of keys of the nodes to be added
   *  - initial_poses: poses of the nodes to be added
   *  - valences: mesh graph vertices that are valences of the nodes
   *  - valence_prefix: prefix of the valences (associated to robot id)
   *  - add_prior: boolean - add a Prior Factor or not
   *  - edge_variance: covariance of the node to mesh edges
   *  - prior_variance: if add prior, covariance on the nodes
   */
  void addNewTempNodesValences(const std::vector<gtsam::Key>& keys,
                               const std::vector<gtsam::Pose3>& initial_poses,
                               const std::vector<Vertices>& valences,
                               const char& valence_prefix,
                               bool add_prior,
                               double edge_variance = 1e-2,
                               const std::vector<gtsam::Pose3>& initial_guesses = {},
                               double prior_variance = 1e-8);

  /*! \brief Add a new between factor to the deformation graph
   *  - key_from: Key of front node to connect between factor
   *  - key_to: Key of back node to connect between factor
   *  - meas: Measurement of between factor
   *  - Estimated position of new node (the back node if node is new)
   *  - variance: covariance on the between factor
   */
  void addNewBetween(const gtsam::Key& key_from,
                     const gtsam::Key& key_to,
                     const gtsam::Pose3& meas,
                     const gtsam::Pose3& initial_pose = gtsam::Pose3(),
                     double variance = 1e-4);

  /*! \brief Add a new temporary between factor to the deformation graph
   *  - key_from: Key of front node to connect between factor
   *  - key_to: Key of back node to connect between factor
   *  - meas: Measurement of between factor
   *  - Estimated position of new node (the back node if node is new)
   *  - variance: covariance on the temporary between factor
   */
  void addNewTempBetween(const gtsam::Key& key_from,
                         const gtsam::Key& key_to,
                         const gtsam::Pose3& meas,
                         const gtsam::Pose3& initial_pose = gtsam::Pose3(),
                         double variance = 1e-2);

  /*! \brief Add new edges as temporary between factor to the deformation graph
   *  - edges: pose_graph_tools::PoseGraph type with the edges to add
   *  - variance: covariance on the added temp edges
   */
  void addNewTempEdges(const pose_graph_tools::PoseGraph& edges,
                       double variance = 1e-2,
                       bool rotations_known = true);

  /*! \brief Add a new mesh edge to deformation graph
   *  - mesh_edges: edges storing key-key pairs
   *  - mesh_nodes: gtsam values encoding key value pairs of new nodes
   *  - added_indices: indices of nodes that was successfully added
   *  - variance: covariance of the deformation graph edges
   *  - optimize: optimize or just add to pgo
   */
  void addNewMeshEdgesAndNodes(
      const std::vector<std::pair<gtsam::Key, gtsam::Key> >& mesh_edges,
      const gtsam::Values& mesh_nodes,
      const std::unordered_map<gtsam::Key, ros::Time>& node_stamps,
      std::vector<size_t>* added_indices,
      std::vector<ros::Time>* added_index_stamps,
      double variance = 1e-4);

  /*! \brief Add connections from a pose graph node to mesh vertices nodes
   *  - key: Key of pose graph node
   *  - valences: The mesh vertices nodes to connect to
   *  - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   *  - variance: covariance of the deformation graph edges
   *  - optimize: optimize or just add to pgo
   */
  void addNodeValence(const gtsam::Key& key,
                      const Vertices& valences,
                      const char& valence_prefix,
                      double variance = 1e-4);

  /*! \brief Add temporary connections from a pose graph node to mesh vertices
   * nodes
   *  - key: Key of pose graph node
   *  - valences: The mesh vertices nodes to connect to
   *  - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   *  - variance: covariance of the deformation graph edges
   *  - optimize: optimize or just add to pgo
   */
  void addTempNodeValence(const gtsam::Key& key,
                          const Vertices& valences,
                          const char& valence_prefix,
                          double variance = 1e-2);

  /*! \brief Remove sll prior factors of nodes that have given prefix
   *  - prefix: prefix of nodes to remove prior
   */
  void removePriorsWithPrefix(const char& prefix);

  /*! \brief Get the optimized estimates for nodes with certain prefix
   *  - prefix: prefix of the nodes to query best estimate
   */
  std::vector<gtsam::Pose3> getOptimizedTrajectory(char prefix) const;

  /*! \brief Get the current (including unoptimized) estimates for nodes with certain
   * prefix
   *  - prefix: prefix of the nodes to query estimate
   */
  std::vector<gtsam::Pose3> getQueuedTrajectory(char prefix) const;

  /*! \brief Deform a mesh based on the deformation graph
   * - original_mesh: mesh to deform
   * - stamps: timestamp of vertices in mesh to deform
   * - prefix: the prefixes of the key of the nodes corresponding to mesh
   * - k: how many nearby nodes to use to adjust new position of vertices when
   * interpolating for deformed mesh
   * - tol_t: largest difference in time such that a control point can be
   * considered for association
   */
  pcl::PolygonMesh deformMesh(const pcl::PolygonMesh& original_mesh,
                              const std::vector<ros::Time>& stamps,
                              const std::vector<int>& graph_indices,
                              const char& prefix,
                              size_t k = 4,
                              double tol_t = 10.0);

  /*! \brief Deform a mesh based on the deformation graph
   * - original_mesh: mesh to deform
   * - stamps: timestamp of vertices in mesh to deform
   * - prefix: the prefixes of the key of the nodes corresponding to mesh
   * - optimized_values: values of the optimized control points
   * - k: how many nearby nodes to use to adjust new position of vertices when
   * interpolating for deformed mesh
   * - tol_t: largest difference in time such that a control point can be
   * considered for association
   */
  pcl::PolygonMesh deformMesh(const pcl::PolygonMesh& original_mesh,
                              const std::vector<ros::Time>& stamps,
                              const std::vector<int>& graph_indices,
                              const char& prefix,
                              const gtsam::Values& optimized_values,
                              size_t k = 4,
                              double tol_t = 10.0);

  /*! \brief Deform a mesh vertices based on the deformation graph
   * - original_vertices: undeformed vertices
   * - stamps: timestamp of vertices in mesh to deform
   * - prefix: the prefixes of the key of the nodes corresponding to mesh
   * - optimized_values: values of the optimized control points
   * - new_vertices: deformed vertices
   * - k: how many nearby nodes to use to adjust new position of vertices when
   * interpolating for deformed mesh
   * - tol_t: largest difference in time such that a control point can be
   * considered for association
   */
  void deformPoints(pcl::PointCloud<pcl::PointXYZRGBA>& new_vertices,
                    const pcl::PointCloud<pcl::PointXYZRGBA>& original_vertices,
                    const std::vector<ros::Time>& stamps,
                    char prefix,
                    const gtsam::Values& optimized_values,
                    size_t k = 4,
                    double tol_t = 10.0,
                    const std::vector<int>* graph_indices = nullptr,
                    int start_index_hint = -1,
                    std::vector<std::set<size_t>>* vertex_graph_map = nullptr);

  /*! \brief Get the number of loop closures processed by pgo
   */
  inline size_t getNumLoopclosures() const { return pgo_->getNumLC(); }

  /*! \brief Get the GNC weights from optimization
   */
  inline gtsam::Vector getGncWeights() const { return pgo_->getGncWeights(); }

  inline gtsam::Vector getTempFactorGncWeights() const {
    return pgo_->getGncTempWeights();
  }

  inline gtsam::Vector getAllGncWeights() const { return pgo_->getGncWeights(); }

  /*! \brief Get the number of mesh vertices nodes in the deformation graph
   * - outputs the number of mesh vertices nodes
   */
  inline size_t getNumVertices() const {
    size_t num_vertices = 0;
    for (const auto& pfx_vertices : vertex_positions_) {
      num_vertices += pfx_vertices.second.size();
    }
    return num_vertices;
  }

  /*! \brief Gets the estimated values since last optimization
   *  - outputs last estimated values as GTSAM Values
   */
  inline const gtsam::Values& getGtsamValues() const { return values_; }

  /*! \brief Gets the factors added to the backend, minus the detected outliers
   *  - outputs the factors as a GTSAM NonlinearFactorGraph
   */
  inline gtsam::NonlinearFactorGraph getGtsamFactors() const { return nfg_; }

  /*! \brief Gets the new estimated values
   *  - outputs new estimated values as GTSAM Values
   */
  inline gtsam::Values getGtsamNewValues() const { return new_values_; }

  /*! \brief Gets the new factors added
   *  - outputs the new factors as a GTSAM NonlinearFactorGraph
   */
  inline gtsam::NonlinearFactorGraph getGtsamNewFactors() const { return new_factors_; }

  /*! \brief Gets the pose graph from the backend
   *   - timestamps: map of robot id to sequential timestamps in order to stamp
   * the nodes in the output pose graph msg
   *  - outputs the pose graph in pose_graph_tools PoseGraph type
   */
  inline GraphMsgPtr getPoseGraph(
      const std::map<size_t, std::vector<ros::Time> >& timestamps) const {
    return GtsamGraphToRos(nfg_, values_, timestamps, gnc_weights_);
  }

  /*! \brief Get the consistency factors (ie. the deformation edge factors)
   */
  inline gtsam::NonlinearFactorGraph getConsistencyFactors() const {
    return consistency_factors_;
  }

  /*! \brief Get the intial pose of a keyframe node
   */
  inline gtsam::Pose3 getInitialPose(const char& prefix, const size_t& index) const {
    return pg_initial_poses_.at(prefix).at(index);
  }

  /*! \brief Get the intial position of a vertex
   */
  inline gtsam::Point3 getInitialPositionVertex(const char& prefix,
                                                const size_t& index) const {
    return vertex_positions_.at(prefix).at(index);
  }

  /*! \brief Get the intial positions of the vertices corresponding to prefix
   */
  inline std::vector<gtsam::Point3> getInitialPositionsVertices(
      const char& prefix) const {
    return vertex_positions_.at(prefix);
  }

  inline bool hasVertexKey(char prefix) const {
    return vertex_positions_.count(prefix);
  }

  /*! \brief Set whether or not to force vertex recalculation
   */
  inline void setForceRecalculate(bool force_recalculate) {
    force_recalculate_ = force_recalculate;
  }

  /*! \brief Recalculate vertices getter
   */
  inline bool getRecalculateVertices() { return recalculate_vertices_; }

  /*! \brief Recalculate vertices setter
   */
  inline void setRecalculateVertices() { recalculate_vertices_ = true; }

  /*! \brief Gets the temp values since last optimization
   *  - outputs last temp values as GTSAM Values
   */
  inline const gtsam::Values& getGtsamTempValues() const { return temp_values_; }

  /*! \brief Gets the temp factors added to the backend, minus the detected
   * outliers
   *  - outputs the factors as a GTSAM NonlinearFactorGraph
   */
  inline gtsam::NonlinearFactorGraph getGtsamTempFactors() const { return temp_nfg_; }

  /*! \brief Clear all temporary values, factors, and related structures
   */
  inline void clearTemporaryStructures() {
    temp_values_ = gtsam::Values();
    temp_nfg_ = gtsam::NonlinearFactorGraph();
    pgo_->clearTempFactorsValues();
    temp_pg_initial_poses_.clear();
  }

  inline const KimeraRPGO::RobustSolverParams& getParams() const { return pgo_params_; }

  /*! \brief Force an optimization of the deformation graph without adding new
   * factors
   */
  void optimize();

  /*! \brief Add and update the new factors and values. Let RPGO naturally decide to
   * optimize or not
   */
  void update();

  /*! \brief Update the values. Use to update initial estimate. Use with caution since
   * initial estimate and result shares same variable. (only depends on if you call
   * before or after optimize)
   */
  void updateValues(const gtsam::Values& updates);

  /*! \brief Set RPGO parameters
   */
  void setParams(const KimeraRPGO::RobustSolverParams& params);

  /*! \brief Gets the temp values since last optimization
   *  - min: get values with key above this value
   *  - max: get values with key less than this value
   *  - outputs last temp values as GTSAM Values
   */
  void getGtsamTempValuesFiltered(gtsam::Values* values,
                                  const gtsam::Key& min,
                                  const gtsam::Key& max) const;

  /*! \brief Gets the temp factors added to the backend, minus the detected
   * outliers
   *  - min: get factors with at least one key above this value
   *  - max: get factors with both keys below this value
   *  - outputs the factors as a GTSAM NonlinearFactorGraph
   */
  void getGtsamTempFactorsFiltered(gtsam::NonlinearFactorGraph* nfg,
                                   const gtsam::Key& min,
                                   const gtsam::Key& max) const;

  /*! \brief Save deformation graph to file
   * - filename: output file name
   */
  void save(const std::string& filename) const;

  /*! \brief Load deformation graph from file
   * - filename: input file name
   */
  void load(const std::string& filename,
            bool include_temp = true,
            bool set_robot_id = false,
            size_t new_robot_id = 0);

  inline bool hasPrefixPoses(char prefix) const {
    return pg_initial_poses_.count(prefix);
  }

  size_t findStartIndex(char prefix,
                        int start_index_hint,
                        const std::vector<ros::Time>& stamps,
                        double tol_t) const;

  void predeformPoints(pcl::PointCloud<pcl::PointXYZRGBA>& new_vertices,
                       const pcl::PointCloud<pcl::PointXYZRGBA>& old_vertices,
                       const gtsam::Values& optimized_values,
                       const std::vector<int>& graph_indices,
                       std::vector<size_t>& indices_to_deform,
                       char prefix,
                       size_t start_index);

 private:
  bool verbose_;

  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;

  // Keep track of vertices not part of mesh
  // for embedding trajectory, etc.
  std::map<char, std::vector<gtsam::Pose3> > pg_initial_poses_;
  std::unordered_map<gtsam::Key, gtsam::Pose3> temp_pg_initial_poses_;

  std::map<char, std::vector<gtsam::Point3> > vertex_positions_;
  std::map<char, std::vector<ros::Time> > vertex_stamps_;

  KimeraRPGO::RobustSolverParams pgo_params_;
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;

  // factors
  gtsam::NonlinearFactorGraph nfg_;
  // current estimate
  gtsam::Values values_;
  // temp factors
  gtsam::NonlinearFactorGraph temp_nfg_;
  // current estimate for temp nodes
  gtsam::Values temp_values_;
  // gnc weights
  gtsam::Vector gnc_weights_;

  // new factors and values
  gtsam::NonlinearFactorGraph new_factors_;
  gtsam::Values new_values_;

  //// Below separated factor types for debugging
  // factor graph encoding the mesh structure
  gtsam::NonlinearFactorGraph consistency_factors_;

  // Force deformation of vertices every optimization
  bool force_recalculate_;

  // Recalculate only if new measurements added
  bool recalculate_vertices_;
  std::map<char, pcl::PointCloud<pcl::PointXYZRGBA> > last_calculated_vertices_;
};

typedef std::shared_ptr<DeformationGraph> DeformationGraphPtr;

/*! \brief fill rviz markers for the deformation graph
 * - graph: deformation graph to make the message for
 * - stamp: ros timestamp of messages
 * - mesh_mesh_viz: marker for mesh-mesh edges
 * - pose_mesh_viz: marker for pose-mesh edges
 */
void fillDeformationGraphMarkers(const DeformationGraph& graph,
                                 const ros::Time& stamp,
                                 visualization_msgs::Marker& mesh_mesh_viz,
                                 visualization_msgs::Marker& pose_mesh_viz);

}  // namespace kimera_pgmo
