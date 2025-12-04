/**
 * @file   deformation_graph.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pose_graph_tools/pose_graph.h>

#include <map>
#include <unordered_map>
#include <vector>

#include "kimera_pgmo/mesh_deformation.h"
#include "kimera_pgmo/pcl_mesh_traits.h"
#include "kimera_pgmo/utils/common_functions.h"
#include "kimera_pgmo/utils/common_structs.h"
#include "kimera_pgmo/utils/logging.h"
#include "kimera_pgmo/utils/range_generator.h"

namespace kimera_pgmo {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using GtsamJacobianType = boost::optional<gtsam::Matrix&>;
#define JACOBIAN_DEFAULT \
  {}
#else
using GtsamJacobianType = gtsam::OptionalMatrixType;
#define JACOBIAN_DEFAULT nullptr
#endif

/*
 * @brief Struct containing necessary info to add valence factor for a node
 */
struct NodeValenceInfo {
  char valence_prefix;
  gtsam::Key key;
  gtsam::Pose3 pose;
  Vertices valence;
};

using NodeValenceInfoList = std::vector<NodeValenceInfo>;

using EdgeTypeVarianceMap = std::map<pose_graph_tools::PoseGraphEdge::Type, double>;

/*! \brief Define a factor type for edges between two mesh vertices or between a
 * mesh vertex and a pose graph node to be added to deformation graph. Inherited
 * from GTSAM NoiseModelFactor2
 */
class DeformationEdgeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  gtsam::Point3 measurement_;

 public:
  DeformationEdgeFactor(gtsam::Key node1_key,
                        gtsam::Key node2_key,
                        const gtsam::Point3& measurement,
                        gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
            model, node1_key, node2_key),
        measurement_(measurement) {}

  DeformationEdgeFactor(gtsam::Key node1_key,
                        gtsam::Key node2_key,
                        const gtsam::Pose3& node1_pose,
                        const gtsam::Point3& node2_point,
                        gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
            model, node1_key, node2_key) {
    measurement_ =
        node1_pose.rotation().inverse().rotate(node2_point - node1_pose.translation());
  }

  virtual ~DeformationEdgeFactor() {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1,
                              const gtsam::Pose3& p2,
                              GtsamJacobianType H1 = JACOBIAN_DEFAULT,
                              GtsamJacobianType H2 = JACOBIAN_DEFAULT) const override {
    // position of node 2 in frame of node 1
    gtsam::Matrix H_R1, H_t1, H_t2;
    gtsam::Rot3 R1 = p1.rotation();
    gtsam::Point3 t1 = p1.translation(H_t1);
    // New position of node 2 according to deformation p1 of node 1
    gtsam::Point3 t2_1 = t1 + R1.rotate(measurement_, H_R1);
    gtsam::Point3 t2_2 = p2.translation(H_t2);

    // Calculate Jacobians
    if (H1) {
      Eigen::MatrixXd Jacobian_1 = Eigen::MatrixXd::Zero(3, 6);
      Jacobian_1.block<3, 3>(0, 0) = H_R1;
      Jacobian_1 = Jacobian_1 + H_t1;
      *H1 = Jacobian_1;
    }

    if (H2) {
      Eigen::MatrixXd Jacobian_2 = Eigen::MatrixXd::Zero(3, 6);
      Jacobian_2 = Jacobian_2 - H_t2;
      *H2 = Jacobian_2;
    }

    return t2_1 - t2_2;
  }

  inline gtsam::Point3 measurement() const { return measurement_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new DeformationEdgeFactor(*this));
  }
};

#undef JACOBIAN_DEFAULT

class DeformationGraph {
 public:
  /*! \brief Deformation graph class constructor
   */
  DeformationGraph(bool add_init_vertex_prior = false);
  ~DeformationGraph();

  inline void setVerboseFlag(bool verbose) { verbose_ = verbose; }

  /*! \brief Directly add a full pose graph to the deformation graph
   *  - pose_graph: full pose graph
   *  - variance_map: map edge type to variance value
   *  - robot_id_remap: robot id remapping
   */
  void processPoseGraph(const pose_graph_tools::PoseGraph& pose_graph,
                        const EdgeTypeVarianceMap& variance_map,
                        std::map<size_t, size_t> robot_id_remap = {},
                        const Eigen::Isometry3d* const transform = nullptr);

  /*! \brief Directly add a full mesh graph to the deformation graph. TODO(Yun) figure
   * out how we handle the pose graph node to mesh graph node connections.
   *  - mesh_graph: full mesh graph
   *  - variance_map: map edge type to variance value
   *  - robot_id_remap: robot id remapping
   */
  void processMeshGraph(const pose_graph_tools::PoseGraph& mesh_graph,
                        const EdgeTypeVarianceMap& variance_map,
                        std::map<size_t, size_t> robot_id_remap = {},
                        const Eigen::Isometry3d* const transform = nullptr);

  /*! \brief Fix the measurements of multiple nodes
   *  - measurements: a vector of key->pose pair of node measurements
   *  - variance: covariance of the prior factors
   */
  void processNodeMeasurements(
      const std::vector<std::pair<gtsam::Key, gtsam::Pose3>>& measurements,
      double variance = 1e-4);

  /*! \brief Initialize with new node of a trajectory
   *  - key: Key of first node in new trajectory
   *  - initial_pose: Initial measurement of first node
   *  - add_prior: boolean - add a Prior Factor or not
   *  - prior_variance: covariance of the prior
   */
  void processNewNode(const gtsam::Key& key,
                      const gtsam::Pose3& initial_pose,
                      bool add_prior,
                      double prior_variance = 1e-8);

  /*! \brief Initialize with new node of a trajectory, but keep it temporary
   *  - key: Key of first node in new trajectory
   *  - initial_pose: Initial measurement of first node
   *  - add_prior: boolean - add a Prior Factor or not
   *  - prior_variance: covariance of the prior
   */
  void processNewTempNode(const gtsam::Key& key,
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
  void processNewTempNodesValences(const NodeValenceInfoList& factors,
                                   bool add_prior,
                                   double edge_variance = 1e-2,
                                   double prior_variance = 1e-8);

  /*! \brief Add a new between factor to the deformation graph
   *  - key_from: Key of front node to connect between factor
   *  - key_to: Key of back node to connect between factor
   *  - meas: Measurement of between factor
   *  - variance: covariance on the between factor
   */
  void processNewBetween(const gtsam::Key& key_from,
                         const gtsam::Key& key_to,
                         const gtsam::Pose3& meas,
                         double variance = 1e-4);

  /*! \brief Adding new initial guess according to an odometry measurement
   *  - key_from: Key of front node to connect between factor
   *  - key_to: Key of back node to connect between factor
   *  - meas: Measurement of between (odom) factor
   */
  void updatePoseGraphInitialGuess(const gtsam::Key& key_from,
                                   const gtsam::Key& key_to,
                                   const gtsam::Pose3& meas);

  /*! \brief Add a new temporary between factor to the deformation graph
   *  - key_from: Key of front node to connect between factor
   *  - key_to: Key of back node to connect between factor
   *  - meas: Measurement of between factor
   *  - variance: covariance on the temporary between factor
   */
  void processNewTempBetween(const gtsam::Key& key_from,
                             const gtsam::Key& key_to,
                             const gtsam::Pose3& meas,
                             double variance = 1e-4);

  /*! \brief Add new edges as temporary between factor to the deformation graph
   *  - edges: pose_graph_tools::PoseGraph type with the edges to add
   *  - variance: covariance on the added temp edges
   */
  void processNewTempEdges(const pose_graph_tools::PoseGraph& edges,
                           double variance = 1e-2);

  /*! \brief Add a new mesh edge to deformation graph
   *  - mesh_edges: edges storing key-key pairs
   *  - mesh_nodes: gtsam values encoding key value pairs of new nodes
   *  - added_indices: indices of nodes that was successfully added
   *  - variance: covariance of the deformation graph edges
   */
  void processNewMeshEdgesAndNodes(
      const std::vector<std::pair<gtsam::Key, gtsam::Key>>& mesh_edges,
      const gtsam::Values& mesh_nodes,
      const std::unordered_map<gtsam::Key, Timestamp>& node_stamps,
      std::vector<size_t>* added_indices,
      std::vector<Timestamp>* added_index_stamps,
      double variance = 1e-4);

  /*! \brief Add connections from a pose graph node to mesh vertices nodes
   *  - key: Key of pose graph node
   *  - valences: The mesh vertices nodes to connect to
   *  - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   *  - variance: covariance of the deformation graph edges
   */
  void processNodeValence(const gtsam::Key& key,
                          const Vertices& valences,
                          const char& valence_prefix,
                          double variance = 1e-4,
                          bool temp = false);

  /*! \brief Check before adding node and vertex edge and get node pose and vertex
   * positiong
   *  - key: Key of pose graph node
   *  - vertex: Key of vertex
   *  - node_pose: reference to node pose
   *  - vertex_pos: reference to vertex position
   */
  bool checkNodeValence(const gtsam::Key& key,
                        const gtsam::Key& vertex,
                        gtsam::Pose3& node_pose,
                        gtsam::Point3& vertex_pos) const;

  /*! \brief Add deformation graph edge between mesh vertices
   *  - source_pose: pose of source (for the between) that is in same frame as the
   * initial vertex positions
   *  - source: The source set of mesh vertices
   *  - dest_pose: pose of dest (for the between) that is in the same frame as the
   * initial vertex positions
   *  - dest: The destination set of mesh vertices
   *  - source_T_dest: measurement to add (of the between)
   *  - source_prefix: the prefixes of the key of the nodes corresponding to source mesh
   * vertices
   *  - dest_prefix: the prefixes of the key of the nodes corresponding to the dest mesh
   * vertices
   *  - variance: covariance of the deformation graph edges
   */
  void processBetweenAsMeshConnections(const gtsam::Pose3& source_pose,
                                       const Vertices& source,
                                       const gtsam::Pose3& dest_pose,
                                       const Vertices& dest,
                                       const gtsam::Pose3& source_T_dest,
                                       const char& source_prefix,
                                       const char& dest_prefix,
                                       double variance = 1e-4,
                                       bool temp = false,
                                       bool known_inliers = false);

  /*! \brief Add point measurements as a deformation edge factor
   *  - from_key: key of the pose point measurement is made from
   *  - to_key: key of the point
   *  - from_pose: pose where measuremet is made in deformation graph frame
   *  - to_point: point measurement in deformation graph frame
   *  - variance: covariance of the deformation graph edges
   *  - temp: temporary factor
   */
  void processPointMeasurement(const gtsam::Key& from_key,
                               const gtsam::Key& to_key,
                               const gtsam::Pose3& from_pose,
                               const gtsam::Point3& to_point,
                               double variance,
                               bool temp = false,
                               bool known_inlier = false);

  /*! \brief Remove sll prior factors of nodes that have given prefix
   *  - prefix: prefix of nodes to remove prior
   */
  void removePriorsWithPrefix(const char& prefix);

  /*! \brief Get the optimized estimates for nodes with certain prefix
   *  - prefix: prefix of the nodes to query best estimate
   */
  std::vector<gtsam::Pose3> getTrajectory(char prefix) const;

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
                              const std::vector<Timestamp>& stamps,
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
                              const std::vector<Timestamp>& stamps,
                              const std::vector<int>& graph_indices,
                              const char& prefix,
                              const gtsam::Values& optimized_values,
                              size_t k = 4,
                              double tol_t = 10.0);

  /*! \brief Deform mesh vertices based on the deformation graph
   * - vertices: vertices to deform
   * - original_vertices: undeformed vertices
   * - prefix: the prefixes of the key of the nodes corresponding to mesh
   * - optimized_values: values of the optimized control points
   * - new_vertices: deformed vertices
   * - k: how many nearby nodes to use to adjust new position of vertices when
   * interpolating for deformed mesh
   * - tol_t: largest difference in time such that a control point can be
   * considered for association
   */
  template <typename CloudIn, typename CloudOut>
  void deformPoints(CloudOut& vertices,
                    const CloudIn& old_vertices,
                    char prefix,
                    const gtsam::Values& optimized_values,
                    size_t k = 4,
                    double tol_t = 10.0,
                    const std::vector<int>* graph_indices = nullptr,
                    int start_index_hint = -1,
                    std::vector<std::set<size_t>>* vertex_graph_map = nullptr);

  /*! \brief Deform mesh vertices based on the deformation graph
   * - vertices: vertices to deform
   * - original_vertices: undeformed vertices
   * - prefix: the prefixes of the key of the nodes corresponding to mesh
   * - k: how many nearby nodes to use to adjust new position of vertices when
   * interpolating for deformed mesh
   * - tol_t: largest difference in time such that a control point can be
   * considered for association
   */
  template <typename CloudIn, typename CloudOut>
  void deformAllPoints(CloudOut& vertices,
                       const CloudIn& old_vertices,
                       char prefix,
                       size_t k = 4,
                       double tol_t = 10.0) const;

  /*! \brief Peform custom deformation on a set of points
   * - callback: processing to perform for each point
   * - original_vertices: undeformed vertices
   * - prefix: the prefixes of the key of the nodes corresponding to mesh
   * - k: how many nearby nodes to use to adjust new position of vertices when
   * interpolating for deformed mesh
   * - tol_t: largest difference in time such that a control point can be
   * considered for association
   */
  template <typename CloudIn>
  void customDeformation(
      const std::function<void(const Eigen::Isometry3d&, size_t)>& callback,
      const CloudIn& points,
      char prefix,
      size_t k,
      double tol_t) const;

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
                    const std::vector<Timestamp>& stamps,
                    char prefix,
                    const gtsam::Values& optimized_values,
                    size_t k = 4,
                    double tol_t = 10.0,
                    const std::vector<int>* graph_indices = nullptr,
                    int start_index_hint = -1,
                    std::vector<std::set<size_t>>* vertex_graph_map = nullptr);

  /*! \brief Get the number of loop closures processed by pgo
   */
  inline size_t getNumLoopclosures() const { return num_loopclosures_; }

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
  const gtsam::Values* getValues() const { return values_.get(); }

  /*! \brief Gets the estimated values since last optimization as a copy
   *  - outputs last estimated values as GTSAM Values
   */
  const gtsam::Values getValuesCopy() const { return *values_; }

  /*! \brief Gets the factors added to the backend
   *  - outputs the factors as a GTSAM NonlinearFactorGraph
   */
  const gtsam::NonlinearFactorGraph* getFactors() const { return nfg_.get(); }

  /*! \brief Gets the factors added to the backend as a copy
   *  - outputs the factors as a GTSAM NonlinearFactorGraph
   */
  const gtsam::NonlinearFactorGraph getFactorsCopy() const { return *nfg_; }

  /*! \brief Gets the set of known inliers
   *  - outputs the set of inlier indices
   */
  const std::set<size_t>* getKnownInlierSet() const { return known_inliers_.get(); }

  /*! \brief Gets the set of known inliers as a copy
   *  - outputs the set of inlier indices
   */
  const std::set<size_t> getKnownInlierSetCopy() const { return *known_inliers_; }

  /*! \brief Gets the temp values since last optimization
   *  - outputs last temp values as GTSAM Values
   */
  const gtsam::Values* getTempValues() const { return temp_values_.get(); }

  /*! \brief Gets the temp values since last optimization as a copy
   *  - outputs last temp values as GTSAM Values
   */
  const gtsam::Values getTempValuesCopy() const { return *temp_values_; }

  /*! \brief Gets the temp factors added to the backend as a copy
   *  - outputs the factors as a GTSAM NonlinearFactorGraph
   */
  const gtsam::NonlinearFactorGraph* getTempFactors() const { return temp_nfg_.get(); }

  /*! \brief Gets the temp factors added to the backend as a copy
   *  - outputs the factors as a GTSAM NonlinearFactorGraph
   */
  const gtsam::NonlinearFactorGraph getTempFactorsCopy() const { return *temp_nfg_; }

  /*! \brief Gets the set of temp known inliers
   *  - outputs the set of inlier indices
   */
  const std::set<size_t>* getTempKnownInlierSet() const {
    return temp_known_inliers_.get();
  }

  /*! \brief Gets copy of the set of temp known inliers
   *  - outputs the set of inlier indices
   */
  const std::set<size_t> getTempKnownInlierSetCopy() const {
    return *temp_known_inliers_;
  }

  /*! \brief Gets the inlier weights since last optimization
   *  - outputs inlier weights as GTSAM vector
   */
  const std::vector<double>* getInlierWeights() const { return inlier_weights_.get(); }

  /*! \brief Gets the temp inlier weights since last optimization
   *  - outputs temp inlier weights as GTSAM vector
   */
  const std::vector<double>* getTempInlierWeights() const {
    return temp_inlier_weights_.get();
  }

  /*! \brief Gets the pose graph from the backend
   *   - timestamps: map of robot id to sequential timestamps in order to stamp
   * the nodes in the output pose graph msg
   *  - outputs the pose graph in pose_graph_tools::PoseGraph type
   */
  pose_graph_tools::PoseGraph::Ptr getPoseGraph(
      const std::map<size_t, std::vector<Timestamp>>& timestamps,
      bool include_deformation_edges = false,
      bool include_between_edges = true,
      bool optimized = true) const;

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

  /*! \brief Get the timestamps of the vertices corresponding to prefix
   */
  inline std::vector<Timestamp> getVertexStamps(const char prefix) const {
    return vertex_stamps_.at(prefix);
  }

  inline bool hasVertexKey(char prefix) const {
    return vertex_positions_.count(prefix);
  }

  /*! \brief Get the observed stamp of a vertex
   */
  inline Timestamp getStampVertex(const char& prefix, const size_t& index) const {
    return vertex_stamps_.at(prefix).at(index);
  }

  /*! \brief Get the observed stamps of the vertices corresponding to prefix
   */
  inline std::vector<Timestamp> getStampVertices(const char& prefix) const {
    return vertex_stamps_.at(prefix);
  }

  /*! \brief Recalculate vertices getter
   */
  inline bool getRecalculateVertices() { return recalculate_vertices_; }

  /*! \brief Recalculate vertices setter
   */
  inline void setRecalculateVertices() { recalculate_vertices_ = true; }

  /*! \brief Clear all (everything)
   */
  inline void clear() {
    nfg_->resize(0);
    temp_nfg_->resize(0);
    values_->clear();
    temp_values_->clear();
    pg_initial_poses_.clear();
    temp_pg_initial_poses_.clear();
    vertex_positions_.clear();
    vertex_stamps_.clear();
  }

  /*! \brief Clear only mesh vertices and their associated factors
   * Preserves robot poses and pose-only factors
   */
  void clearMeshNodesOnly();

  /*! \brief Clear the last mesh interpolation cache
   */
  void clearMeshCache() { last_calculated_vertices_.clear(); }

  /*! \brief Clear all temporary values, factors, and related structures
   */
  inline void clearFactors() {
    nfg_->resize(0);
    temp_nfg_->resize(0);
  }

  /*! \brief Clear all temporary values, factors, and related structures
   */
  inline void clearTemporaryStructures() {
    temp_values_->clear();
    temp_nfg_->resize(0);
    temp_pg_initial_poses_.clear();
  }

  /*! \brief Update the values. Use to update initial estimate. Use with caution since
   * initial estimate and result shares same variable. (only depends on if you call
   * before or after optimize)
   */
  void updateValues(const gtsam::Values& updates);

  /*! \brief Update the temp values. Use to update initial estimate. Use with caution
   * since initial estimate and result shares same variable. (only depends on if you
   * call before or after optimize)
   */
  void updateTempValues(const gtsam::Values& updates);

  /*! \brief Update the inlier weights (e.g. GNC results).
   */
  void updateInlierWeights(const std::vector<double>& weights);

  /*! \brief Update the temp inlier weights (e.g. GNC results).
   */
  void updateTempInlierWeights(const std::vector<double>& weights);

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
            size_t new_robot_id = 0,
            bool include_priors = true);

  inline bool hasPrefixPoses(char prefix) const {
    return pg_initial_poses_.count(prefix);
  }

  template <typename Cloud>
  size_t findStartIndex(char prefix,
                        int start_index_hint,
                        const Cloud& cloud,
                        double tol_t) const;

  template <typename Cloud>
  void fillPreviousPoints(Cloud& vertices, char prefix, size_t start_idx) const;

  template <typename Cloud>
  void cacheNewPoints(const Cloud& vertices, char prefix, size_t start_idx);

  template <typename CloudIn, typename CloudOut>
  void predeformPoints(CloudOut& new_vertices,
                       const CloudIn& vertices,
                       const gtsam::Values& optimized_values,
                       const std::vector<int>& graph_indices,
                       std::vector<size_t>& indices_to_deform,
                       char prefix,
                       size_t start_index);

  inline std::unique_lock<std::mutex> acquireLock() {
    return std::unique_lock<std::mutex>(mutex_);
  }

 protected:
  bool checkNewBetween(const gtsam::Key& key_from, const gtsam::Key& key_to) const;

  void addNewBetween(const gtsam::Key& key_from,
                     const gtsam::Key& key_to,
                     const gtsam::Pose3& meas,
                     double variance,
                     bool temp = false,
                     bool known_inlier = false);

  bool checkNewTempBetween(const gtsam::Key& key_from, const gtsam::Key& key_to) const;

  void addDeformationEdge(const gtsam::Key& from_key,
                          const gtsam::Key& to_key,
                          const gtsam::Pose3& from_pose,
                          const gtsam::Point3& to_point,
                          double variance,
                          bool temp = false,
                          bool known_inlier = false);

  void addDeformationEdge(const gtsam::Key& from_key,
                          const gtsam::Key& to_key,
                          const gtsam::Point3& measurement,
                          double variance,
                          bool temp = false,
                          bool known_inlier = false);

  bool addNewMeshNode(const gtsam::Key& node_key,
                      const gtsam::Pose3& node_pose,
                      const Timestamp& node_stamp);

  bool checkNewMeshNode(const gtsam::Key& node_key) const;

  bool checkNewMeshEdge(const gtsam::Key& from, const gtsam::Key& to) const;

  bool checkNewNode(const gtsam::Key& key) const;

  void addNewNode(const gtsam::Key& key, const gtsam::Pose3& initial_pose);

  void addNewTempNode(const gtsam::Key& key, const gtsam::Pose3& initial_pose);

  void addPrior(const gtsam::Key& key,
                const gtsam::Pose3& pose,
                double variance,
                bool temp = false,
                bool known_inlier = false);

  bool tryConvertFactorToPriorEdge(
      gtsam::NonlinearFactor* factor,
      const std::map<size_t, std::vector<Timestamp>>& timestamps,
      int factor_idx,
      pose_graph_tools::PoseGraphEdge& edge) const;

  bool tryConvertFactorToBetweenEdge(
      gtsam::NonlinearFactor* factor,
      const std::map<size_t, std::vector<Timestamp>>& timestamps,
      int factor_idx,
      pose_graph_tools::PoseGraphEdge& edge) const;

  bool tryConvertFactorToDeformationEdge(gtsam::NonlinearFactor* factor,
                                         pose_graph_tools::PoseGraphEdge& edge) const;

  bool tryConvertKeyToPoseNode(
      const gtsam::Key& key,
      const std::map<size_t, std::vector<Timestamp>>& timestamps,
      pose_graph_tools::PoseGraphNode& node,
      bool optimized = true) const;

  bool tryConvertKeyToMeshNode(const gtsam::Key& key,
                               pose_graph_tools::PoseGraphNode& node,
                               bool optimized = true) const;

  size_t getRemappedId(const std::map<size_t, size_t>& remap, size_t original) const;

  bool checkAdjacency(gtsam::Key from, gtsam::Key to) const;

 private:
  bool add_init_vertex_prior_;
  bool verbose_;

  // Keep track of vertices not part of mesh
  // for embedding trajectory, etc.
  std::map<char, std::vector<gtsam::Pose3>> pg_initial_poses_;
  std::unordered_map<gtsam::Key, gtsam::Pose3> temp_pg_initial_poses_;

  std::map<char, std::vector<gtsam::Point3>> vertex_positions_;
  std::map<char, std::vector<Timestamp>> vertex_stamps_;

  // factors
  std::shared_ptr<gtsam::NonlinearFactorGraph> nfg_;
  // known inlier set
  std::shared_ptr<std::set<size_t>> known_inliers_;
  // current estimate
  std::shared_ptr<gtsam::Values> values_;
  // temp factors
  std::shared_ptr<gtsam::NonlinearFactorGraph> temp_nfg_;
  // known inlier set
  std::shared_ptr<std::set<size_t>> temp_known_inliers_;
  // current temp estimate
  std::shared_ptr<gtsam::Values> temp_values_;
  // gnc weights (from last update)
  std::shared_ptr<std::vector<double>> inlier_weights_;
  // gnc weights for temp factors (from last update)
  std::shared_ptr<std::vector<double>> temp_inlier_weights_;

  // track adjacency
  std::map<gtsam::Key, std::set<gtsam::Key>> adjacency_map_;

  size_t num_loopclosures_ = 0;

  // Recalculate only if new measurements added
  bool recalculate_vertices_;
  std::map<char, pcl::PointCloud<pcl::PointXYZ>> last_calculated_vertices_;

  // Mutex
  std::mutex mutex_;
};

using DeformationGraphPtr = std::shared_ptr<DeformationGraph>;

template <typename Cloud>
size_t DeformationGraph::findStartIndex(char prefix,
                                        int start_index_hint,
                                        const Cloud& cloud,
                                        double tol_t) const {
  const bool have_prefix_vertices =
      last_calculated_vertices_.find(prefix) != last_calculated_vertices_.end();
  if (!have_prefix_vertices) {
    return 0;
  }

  if (recalculate_vertices_) {
    SPARK_LOG(INFO) << "DeformationGraph: Recalculating mesh vertices in deformMesh";
    return 0;
  }

  if (start_index_hint >= 0) {
    return start_index_hint;
  }

  Timestamp min_stamp =
      std::max(static_cast<Timestamp>(0),
               vertex_stamps_.at(prefix).back() - stampFromSec(tol_t));

  RangeGenerator gen(traits::num_vertices(cloud));
  auto bound = std::upper_bound(gen.begin(), gen.end(), min_stamp, [&](auto v, auto i) {
    return v < traits::get_timestamp(cloud, i);
  });

  return std::min(static_cast<size_t>(bound - gen.begin()),
                  last_calculated_vertices_.at(prefix).size());
}

template <typename Cloud>
void DeformationGraph::fillPreviousPoints(Cloud& vertices,
                                          char prefix,
                                          size_t start_idx) const {
  auto iter = last_calculated_vertices_.find(prefix);
  if (iter == last_calculated_vertices_.end()) {
    return;
  }

  for (size_t i = 0; i < start_idx; i++) {
    traits::set_vertex(vertices, i, traits::get_vertex(iter->second, i));
  }
}

template <typename Cloud>
void DeformationGraph::cacheNewPoints(const Cloud& vertices,
                                      char prefix,
                                      size_t start_idx) {
  auto iter = last_calculated_vertices_.find(prefix);
  if (iter == last_calculated_vertices_.end()) {
    iter = last_calculated_vertices_.emplace(prefix, pcl::PointCloud<pcl::PointXYZ>())
               .first;
  }

  const auto total_vertices = traits::num_vertices(vertices);
  iter->second.resize(total_vertices);
  for (size_t i = start_idx; i < total_vertices; i++) {
    traits::set_vertex(iter->second, i, traits::get_vertex(vertices, i));
  }
}

template <typename CloudIn, typename CloudOut>
void DeformationGraph::predeformPoints(CloudOut& new_vertices,
                                       const CloudIn& vertices,
                                       const gtsam::Values& optimized_values,
                                       const std::vector<int>& graph_indices,
                                       std::vector<size_t>& indices_to_deform,
                                       char prefix,
                                       size_t start_idx) {
  const auto num_vertices = traits::num_vertices(vertices);
  for (size_t i = start_idx; i < num_vertices; i++) {
    const int index = graph_indices.at(i);
    if (index < 0 || !optimized_values.exists(gtsam::Symbol(prefix, index))) {
      // Have to check here because sometimes interpolation happen before mesh
      // graph received
      // TODO(yun) double check this
      indices_to_deform.push_back(i);
      continue;
    }

    const Eigen::Vector3d vi = traits::get_vertex(vertices, i).template cast<double>();
    gtsam::Pose3 transform =
        optimized_values.at<gtsam::Pose3>(gtsam::Symbol(prefix, index));
    gtsam::Point3 gindex = vertex_positions_[prefix].at(index);
    gtsam::Point3 deformed_point =
        transform.rotation().rotate(vi - gindex) + transform.translation();
    traits::set_vertex(new_vertices, i, deformed_point.cast<float>());
  }
}

template <typename CloudIn, typename CloudOut>
void DeformationGraph::deformPoints(CloudOut& vertices,
                                    const CloudIn& old_vertices,
                                    char prefix,
                                    const gtsam::Values& optimized_values,
                                    size_t k,
                                    double tol_t,
                                    const std::vector<int>* graph_indices,
                                    int start_index_hint,
                                    std::vector<std::set<size_t>>* vertex_graph_map) {
  // Cannot deform if no nodes in the deformation graph
  if (vertex_positions_.find(prefix) == vertex_positions_.end()) {
    SPARK_LOG(DEBUG)
        << "Deformation graph has no vertices for mesh prefix. No deformation";
    return;
  }

  const auto start_idx = findStartIndex(prefix, start_index_hint, old_vertices, tol_t);
  fillPreviousPoints(vertices, prefix, start_idx);

  std::vector<size_t> to_deform;
  if (start_idx != 0) {
    if (graph_indices) {
      predeformPoints(vertices,
                      old_vertices,
                      optimized_values,
                      *graph_indices,
                      to_deform,
                      prefix,
                      start_idx);
    } else {
      to_deform.resize(traits::num_vertices(vertices) - start_idx);
      std::iota(to_deform.begin(), to_deform.end(), start_idx);
    }
  }

  const std::vector<size_t>* indices_ptr = start_idx == 0 ? nullptr : &to_deform;
  std::vector<std::set<size_t>> vertex_graph_map_deformed;
  deformation::deformPoints(vertices,
                            vertex_graph_map_deformed,
                            old_vertices,
                            prefix,
                            vertex_positions_.at(prefix),
                            vertex_stamps_.at(prefix),
                            optimized_values,
                            k,
                            tol_t,
                            indices_ptr);

  if (vertex_graph_map) {
    if (start_idx == 0) {
      *vertex_graph_map = vertex_graph_map_deformed;
    } else {
      vertex_graph_map->resize(traits::num_vertices(vertices));
      for (size_t i = 0; i < indices_ptr->size(); i++) {
        vertex_graph_map->at(indices_ptr->at(i)) = vertex_graph_map_deformed.at(i);
      }
    }
  }

  cacheNewPoints(vertices, prefix, start_idx);
  recalculate_vertices_ = false;
}

template <typename CloudIn>
void DeformationGraph::customDeformation(
    const std::function<void(const Eigen::Isometry3d&, size_t)>& callback,
    const CloudIn& points,
    char prefix,
    size_t k,
    double tol_t) const {
  // Cannot deform if no nodes in the deformation graph
  if (vertex_positions_.find(prefix) == vertex_positions_.end()) {
    SPARK_LOG(DEBUG)
        << "Deformation graph has no vertices for mesh prefix. No deformation";
    return;
  }

  std::vector<std::set<size_t>> vertex_graph_map_deformed;
  deformation::processPoints(
      [&](const size_t ii,
          std::set<size_t>& control_points_seen,
          char prefix,
          const std::vector<gtsam::Point3>& control_points,
          const gtsam::Values& values,
          const deformation::SearchTree& octree,
          size_t k) {
        const auto transform = interpDeformation(control_points_seen,
                                                 prefix,
                                                 control_points,
                                                 values,
                                                 octree,
                                                 k,
                                                 traits::get_vertex(points, ii));

        callback(transform, ii);
      },
      vertex_graph_map_deformed,
      points,
      prefix,
      vertex_positions_.at(prefix),
      vertex_stamps_.at(prefix),
      *values_,
      k,
      tol_t,
      nullptr);
}

template <typename CloudIn, typename CloudOut>
void DeformationGraph::deformAllPoints(CloudOut& vertices,
                                       const CloudIn& old_vertices,
                                       char prefix,
                                       size_t k,
                                       double tol_t) const {
  // Cannot deform if no nodes in the deformation graph
  if (vertex_positions_.find(prefix) == vertex_positions_.end()) {
    SPARK_LOG(DEBUG)
        << "Deformation graph has no vertices for mesh prefix. No deformation";
    return;
  }

  std::vector<std::set<size_t>> vertex_graph_map_deformed;
  deformation::deformPoints(vertices,
                            vertex_graph_map_deformed,
                            old_vertices,
                            prefix,
                            vertex_positions_.at(prefix),
                            vertex_stamps_.at(prefix),
                            *values_,
                            k,
                            tol_t,
                            nullptr);
}

}  // namespace kimera_pgmo
