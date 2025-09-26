/**
 * @file   deformation_graph.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */
#ifndef KIMERA_PGMO_DEFORMATION_GRAPH_H
#define KIMERA_PGMO_DEFORMATION_GRAPH_H

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pose_graph_tools/pose_graph.h>

#include <filesystem>
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

/*
 * @brief Struct containing necessary info to add valence factor for a node
 */
struct NodeValenceInfo {
  char valence_prefix;
  gtsam::Key key;
  gtsam::Pose3 pose;
  Vertices valence;
};

struct PGOInfo {
 public:
  gtsam::Values values;
  gtsam::NonlinearFactorGraph factors;
  std::set<size_t> known_inliers;
  std::vector<double> inlier_weights;

  void clear();
  void rekey(const std::map<char, char>& prefix_remapping);
  void forceRobotId(size_t robot_id);

  void save(std::ostream& out, bool is_temp = false) const;
  void save(const std::filesystem::path& filepath, bool is_temp = false) const;

  void load(std::istream& in, bool is_temp = false, bool include_priors = true);
  static std::shared_ptr<PGOInfo> load(const std::filesystem::path,
                                       bool is_temp = false,
                                       bool include_priors = true);

 private:
  void rekey(const std::function<gtsam::Symbol(gtsam::Symbol)>& remapping);
};

using NodeValenceInfoList = std::vector<NodeValenceInfo>;
using EdgeTypeVarianceMap = std::map<pose_graph_tools::PoseGraphEdge::Type, double>;

class DeformationGraph {
 public:
  /*! \brief Deformation graph class constructor
   */
  DeformationGraph(bool add_init_vertex_prior = false);
  ~DeformationGraph();

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

  //! \brief Get the number of loop closures processed by pgo
  size_t getNumLoopclosures() const;

  //! \brief Get the number of mesh vertices nodes in the deformation graph
  size_t getNumVertices() const;

  //! \brief Gets the estimated values since last optimization
  const gtsam::Values* getValues() const;

  //! \brief Gets the estimated values since last optimization as a copy
  gtsam::Values getValuesCopy() const;

  //! \brief Gets the factors added to the backend
  const gtsam::NonlinearFactorGraph* getFactors() const;

  //! \brief Gets the factors added to the backend as a copy
  gtsam::NonlinearFactorGraph getFactorsCopy() const;

  //! \brief Gets the set of known inliers
  const std::set<size_t>* getKnownInlierSet() const;

  //! \brief Gets the set of known inliers as a copy
  std::set<size_t> getKnownInlierSetCopy() const;

  //! \brief Gets the temp values since last optimization
  const gtsam::Values* getTempValues() const;

  //! \brief Gets the temp values since last optimization as a copy
  gtsam::Values getTempValuesCopy() const;

  //! \brief Gets the temp factors added to the backend as a copy
  const gtsam::NonlinearFactorGraph* getTempFactors() const;

  //! \brief Gets the temp factors added to the backend as a copy
  gtsam::NonlinearFactorGraph getTempFactorsCopy() const;

  //! \brief Gets the set of temp known inliers
  const std::set<size_t>* getTempKnownInlierSet() const;

  //! \brief Gets copy of the set of temp known inliers
  std::set<size_t> getTempKnownInlierSetCopy() const;

  //! \brief Gets the inlier weights since last optimization
  const std::vector<double>* getInlierWeights() const;

  //! \brief Gets the temp inlier weights since last optimization
  const std::vector<double>* getTempInlierWeights() const;

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

  //! \brief Get the intial pose of a keyframe node
  gtsam::Pose3 getInitialPose(const char& prefix, const size_t& index) const;

  //! \brief Get the intial position of a vertex
  gtsam::Point3 getInitialPositionVertex(const char& prefix, const size_t& index) const;

  //! \brief Get the intial positions of the vertices corresponding to prefix
  std::vector<gtsam::Point3> getInitialPositionsVertices(const char& prefix) const;

  //! \brief Get the timestamps of the vertices corresponding to prefix
  std::vector<Timestamp> getVertexStamps(const char prefix) const;

  bool hasVertexKey(char prefix) const;

  //! \brief Get the observed stamp of a vertex
  Timestamp getStampVertex(const char& prefix, const size_t& index) const;

  //! \brief Get the observed stamps of the vertices corresponding to prefix
  std::vector<Timestamp> getStampVertices(const char& prefix) const;

  //! \brief Recalculate vertices getter
  bool getRecalculateVertices();

  //! \brief Recalculate vertices setter
  void setRecalculateVertices();

  //! \brief Clear all (everything)
  void clear();

  //! \brief Clear all temporary values, factors, and related structures
  void clearFactors();

  //! \brief Clear all temporary values, factors, and related structures
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
   * - include_temp: include all temp values and factors
   * - include_priors: include all prior factors
   */
  void load(const std::string& filename,
            bool include_temp = true,
            bool include_priors = true);

  //! Force all factors and values to use the same robot ID
  void forceRobotId(size_t robot_id);

  bool hasPrefixPoses(char prefix) const;

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
  std::mutex mutex_;

  bool add_init_vertex_prior_;
  size_t num_loopclosures_;

  // deformation graph vertices
  std::map<char, std::vector<gtsam::Point3>> vertex_positions_;
  std::map<char, std::vector<Timestamp>> vertex_stamps_;

  // optimization info
  PGOInfo info_;
  PGOInfo temp_info_;
  std::map<char, std::vector<gtsam::Pose3>> pg_initial_poses_;
  std::unordered_map<gtsam::Key, gtsam::Pose3> temp_pg_initial_poses_;

  // track adjacency
  std::map<gtsam::Key, std::set<gtsam::Key>> adjacency_map_;

  // Recalculate only if new measurements added
  bool recalculate_vertices_;
  std::map<char, pcl::PointCloud<pcl::PointXYZ>> last_calculated_vertices_;
};

using DeformationGraphPtr = std::shared_ptr<DeformationGraph>;

}  // namespace kimera_pgmo

#include "kimera_pgmo/deformation_graph_impl.h"
#endif
