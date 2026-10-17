/**
 * @file   deformation_graph.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <pose_graph_tools/pose_graph.h>

#include <cstdint>
#include <filesystem>
#include <map>
#include <optional>
#include <unordered_map>
#include <vector>

#include "kimera_pgmo/utils/common_structs.h"

namespace kimera_pgmo {

//! Struct containing necessary info to add valence factor for a node
struct NodeValenceInfo {
  char valence_prefix;
  gtsam::Key key;
  gtsam::Pose3 pose;
  std::vector<size_t> valence;
};

struct StampedPose {
  Timestamp timestamp_ns;
  gtsam::Pose3 pose;
};

using NodeValenceInfoList = std::vector<NodeValenceInfo>;
using EdgeTypeVarianceMap = std::map<pose_graph_tools::PoseGraphEdge::Type, double>;

class DeformationGraph {
 public:
  using Ptr = std::shared_ptr<DeformationGraph>;
  using DeformationCallback = std::function<void(const Eigen::Isometry3d&, size_t)>;
  using NodeMeasurements = std::vector<std::pair<gtsam::Key, gtsam::Pose3>>;
  using MeshEdges = std::vector<std::pair<gtsam::Key, gtsam::Key>>;
  using MeshNodeStamps = std::unordered_map<gtsam::Key, Timestamp>;

  DeformationGraph(bool add_init_vertex_prior = false);

  ~DeformationGraph();

  //! Save deformation graph to file
  void save(const std::string& filename) const;

  //! Load deformation graph from file
  static DeformationGraph::Ptr load(const std::filesystem::path& filename,
                                    bool include_priors = true,
                                    std::optional<size_t> new_robot_id = std::nullopt);

  /*! \brief Directly add a full pose graph to the deformation graph
   *  - pose_graph: full pose graph
   *  - variance_map: map edge type to variance value
   *  - robot_id_remap: robot id remapping
   */
  void processPoseGraph(const pose_graph_tools::PoseGraph& pose_graph,
                        const EdgeTypeVarianceMap& variance_map,
                        std::map<size_t, size_t> robot_id_remap = {},
                        const Eigen::Isometry3d* const transform = nullptr);

  /*! \brief Directly add a full mesh graph to the deformation graph.
   *  - mesh_graph: full mesh graph
   *  - variance_map: map edge type to variance value
   *  - robot_id_remap: robot id remapping
   */
  void processMeshGraph(const pose_graph_tools::PoseGraph& mesh_graph,
                        const EdgeTypeVarianceMap& variance_map,
                        std::map<size_t, size_t> robot_id_remap = {},
                        const Eigen::Isometry3d* const transform = nullptr);

  /**
   * Fix the measurements of multiple nodes
   * @param measurements a vector of key->pose pair of node measurements
   * @param variance covariance of the prior factors
   */
  void processNodeMeasurements(const NodeMeasurements& measurements,
                               double variance = 1e-4);

  /**
   * Add a new pose node to the deformation graph
   * @param key Key of pose node
   * @param stamp Timestamp of pose node
   * @param initial_pose Original pose of pose node
   * @param add_prior Add prior factor for pose
   * @param prior_variance Variance for prior factor
   */
  bool addNewNode(gtsam::Key key,
                  Timestamp stamp,
                  const gtsam::Pose3& initial_pose,
                  bool add_prior = false,
                  double prior_variance = 1.0e-8);

  /**
   * Initialize with new node of a trajectory, but keep it temporary
   * @param key Key of first node in new trajectory
   * @param initial_pose Initial measurement of first node
   * @param add_prior Add a Prior Factor or not
   * @param prior_variance Covariance of the prior
   */
  void processNewTempNode(gtsam::Key key,
                          const gtsam::Pose3& initial_pose,
                          bool add_prior,
                          double prior_variance = 1e-8);

  /**
   * Add nodes and their valences, but keep them temporary
   * @param keys Vector of keys of the nodes to be added
   * @param initial_poses Poses of the nodes to be added
   * @param valences Mesh graph vertices that are valences of the nodes
   * @param valence_prefix Prefix of the valences (associated to robot id)
   * @param add_prior Add a Prior Factor or not
   * @param edge_variance Covariance of the node to mesh edges
   * @param prior_variance If add prior, covariance on the nodes
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
  void processNewBetween(gtsam::Key key_from,
                         gtsam::Key key_to,
                         const gtsam::Pose3& meas,
                         double variance = 1e-4);

  /*! \brief Adding new initial guess according to an odometry measurement
   *  - key_from: Key of front node to connect between factor
   *  - key_to: Key of back node to connect between factor
   *  - meas: Measurement of between (odom) factor
   */
  void updatePoseGraphInitialGuess(gtsam::Key key_from,
                                   gtsam::Key key_to,
                                   Timestamp stamp,
                                   const gtsam::Pose3& meas);

  /*! \brief Add a new temporary between factor to the deformation graph
   *  - key_from: Key of front node to connect between factor
   *  - key_to: Key of back node to connect between factor
   *  - meas: Measurement of between factor
   *  - variance: covariance on the temporary between factor
   */
  void processNewTempBetween(gtsam::Key key_from,
                             gtsam::Key key_to,
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
  void processNewMeshEdgesAndNodes(const MeshEdges& mesh_edges,
                                   const gtsam::Values& mesh_nodes,
                                   const MeshNodeStamps& node_stamps,
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
  void processNodeValence(gtsam::Key key,
                          const std::vector<uint64_t>& valences,
                          char valence_prefix,
                          double variance = 1e-4,
                          bool temp = false);

  /*! \brief Check before adding node and vertex edge and get node pose and vertex
   * positiong
   *  - key: Key of pose graph node
   *  - vertex: Key of vertex
   *  - node_pose: reference to node pose
   *  - vertex_pos: reference to vertex position
   */
  bool checkNodeValence(gtsam::Key key,
                        gtsam::Key vertex,
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
                                       const std::vector<uint64_t>& source,
                                       const gtsam::Pose3& dest_pose,
                                       const std::vector<uint64_t>& dest,
                                       const gtsam::Pose3& source_T_dest,
                                       char source_prefix,
                                       char dest_prefix,
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
  void processPointMeasurement(gtsam::Key from_key,
                               gtsam::Key to_key,
                               const gtsam::Pose3& from_pose,
                               const gtsam::Point3& to_point,
                               double variance,
                               bool temp = false,
                               bool known_inlier = false);

  //! Remove all prior factors of nodes that have given prefix
  void removePriorsWithPrefix(char prefix);

  /*! \brief Get the optimized estimates for nodes with certain prefix
   *  - prefix: prefix of the nodes to query best estimate
   */
  std::vector<gtsam::Pose3> getTrajectory(char prefix) const;

  /**
   * Deform vertices based on the deformation graph
   * @param vertices Vertices to deform
   * @param old_vertices Undeformed vertices
   * @param prefix The prefixes of the key of the nodes corresponding to mesh
   * @param k Number of control points to use for deformation
   * @param tol_t Largest time difference to control point for association
   */
  template <typename CloudIn, typename CloudOut>
  void deformPoints(CloudOut& vertices,
                    const CloudIn& old_vertices,
                    char prefix,
                    size_t k = 4,
                    double tol_t = 10.0,
                    int start_index = 0) const;

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
  void customDeformation(const DeformationCallback& callback,
                         const CloudIn& points,
                         char prefix,
                         size_t k = 4,
                         double tol_t = 10.0) const;

  //! Get the number of loop closures processed by pgo
  size_t getNumLoopclosures() const { return num_loopclosures_; }

  //! Get the number of mesh vertices nodes in the deformation graph
  size_t getNumVertices() const;

  //! Gets the estimated values since last optimization
  const gtsam::Values* getValues() const { return &values_; }

  //! Gets the estimated values since last optimization as a copy
  gtsam::Values getValuesCopy() const { return values_; }

  //! Gets the factors added to the backend
  const gtsam::NonlinearFactorGraph* getFactors() const { return &nfg_; }

  //! Gets the factors added to the backend as a copy
  gtsam::NonlinearFactorGraph getFactorsCopy() const { return nfg_; }

  //! Gets the set of known inliers
  const std::set<size_t>* getKnownInlierSet() const { return &known_inliers_; }

  //! Gets the set of known inliers as a copy
  std::set<size_t> getKnownInlierSetCopy() const { return known_inliers_; }

  //! Gets the temp values since last optimization
  const gtsam::Values* getTempValues() const { return &temp_values_; }

  //! Gets the temp values since last optimization as a copy
  gtsam::Values getTempValuesCopy() const { return temp_values_; }

  //! Gets the temp factors added to the backend as a copy
  const gtsam::NonlinearFactorGraph* getTempFactors() const { return &temp_nfg_; }

  //! Gets the temp factors added to the backend as a copy
  gtsam::NonlinearFactorGraph getTempFactorsCopy() const { return temp_nfg_; }

  //! Gets the set of temp known inliers
  const std::set<size_t>* getTempKnownInlierSet() const { return &temp_known_inliers_; }

  //! Gets copy of the set of temp known inliers
  std::set<size_t> getTempKnownInlierSetCopy() const { return temp_known_inliers_; }

  //! Gets the inlier weights since last optimization
  const std::vector<double>* getInlierWeights() const { return &inlier_weights_; }

  //! Gets the temp inlier weights since last optimization
  const std::vector<double>* getTempInlierWeights() const {
    return &temp_inlier_weights_;
  }

  //! Gets the pose graph corresponding to the underlying factor graph
  pose_graph_tools::PoseGraph::Ptr getPoseGraph(bool include_deformation_edges = false,
                                                bool include_between_edges = true,
                                                bool optimized = true) const;

  //! Get the intial pose of a keyframe node
  gtsam::Pose3 getInitialPose(char prefix, size_t index) const;

  //! Get the intial position of a vertex
  gtsam::Point3 getInitialPositionVertex(char prefix, size_t index) const;

  std::vector<size_t> findClosestMeshIndices(size_t robot_id, Timestamp stamp) const;

  //! Clear all (everything)
  void clear();

  //! Clear only mesh vertices and their associated factors
  void clearMeshNodesOnly();

  //! Clear all temporary values, factors, and related structures
  void clearFactors();

  //! Clear all temporary values, factors, and related structures
  void clearTemporaryStructures();

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

  //! Update the inlier weights (e.g. GNC results).
  void updateInlierWeights(const std::vector<double>& weights);

  //! Update the temp inlier weights (e.g. GNC results).
  void updateTempInlierWeights(const std::vector<double>& weights);

  bool hasVertexKey(char prefix) const;
  bool hasPrefixPoses(char prefix) const;

  std::unique_lock<std::mutex> acquireLock();

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

  void addNewTempNode(const gtsam::Key& key, const gtsam::Pose3& initial_pose);

  void addPrior(const gtsam::Key& key,
                const gtsam::Pose3& pose,
                double variance,
                bool temp = false,
                bool known_inlier = false);

  size_t getRemappedId(const std::map<size_t, size_t>& remap, size_t original) const;

  bool checkAdjacency(gtsam::Key from, gtsam::Key to) const;

 private:
  friend struct DGRFLoader;

  std::mutex mutex_;
  bool add_init_vertex_prior_;
  size_t num_loopclosures_;

  std::map<char, std::vector<StampedPose>> trajectories_;
  std::map<char, std::vector<StampedPoint>> control_points_;
  std::unordered_map<gtsam::Key, gtsam::Pose3> temp_pg_initial_poses_;

  gtsam::NonlinearFactorGraph nfg_;
  std::set<size_t> known_inliers_;
  gtsam::Values values_;
  std::vector<double> inlier_weights_;

  gtsam::NonlinearFactorGraph temp_nfg_;
  std::set<size_t> temp_known_inliers_;
  gtsam::Values temp_values_;
  std::vector<double> temp_inlier_weights_;

  std::map<gtsam::Key, std::set<gtsam::Key>> adjacency_map_;
};

}  // namespace kimera_pgmo

#include "kimera_pgmo/impl/deformation_graph.h"  // IWYU pragma: keep
