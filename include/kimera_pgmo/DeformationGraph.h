/**
 * @file   DeformationGraph.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */
#pragma once

#include <map>
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

  gtsam::Vector evaluateError(
      const gtsam::Pose3& p1,
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
};

class DeformationGraph {
 public:
  /*! \brief Deformation graph class constructor
   */
  DeformationGraph();
  ~DeformationGraph();

  /*! \brief Initialize deformation graph along with robust solver backend.
   *  - pgo_trans_threshold: translation threshold (meters of error per node)
   * for backend PCM outlier rejection
   *  - pgo_rot_threshold: rotation threshold (radians of error per node) for
   * backend PCM outlier rejection
   */
  bool initialize(double pgo_trans_threshold, double pgo_rot_threshold);

  /*! \brief Add new vertices as nodes in deformation graph and the sides of the
   * new surfaces as edges in deformation graph.
   *  - new_vertices: new vertices of sampled mesh that makes up most of
   * deformation graph
   *  - new_surfaces: new surfaces of sampled mesh
   *  - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   */
  void updateMesh(const pcl::PointCloud<pcl::PointXYZRGBA>& new_vertices,
                  const std::vector<pcl::Vertices> new_surfaces,
                  const char& prefix);

  /*! \brief Fix the transform of a node corresponding to a sampled mesh vertex
   * in deformation graph. Note that all vertices has an original rotation of
   * identity.
   *  - v: Vertex (index) to impose transform on
   *  - transform: imposed transform (geometry_msgs Pose)
   *  - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   */
  void addMeasurement(const Vertex& v,
                      const geometry_msgs::Pose& transform,
                      const char& prefix);

  /*! \brief Fix the transform of a node corresponding to a pose graph node
   *  - key: Key of the node to transform. Note that this is the key after
   * adding the prefix, etc gtsam::Symbol(prefix, index).key()
   *  - pose: Transform to impose (GTSAM Pose3)
   */
  void addNodeMeasurement(const gtsam::Key& key, const gtsam::Pose3 pose);

  /*! \brief Fix the measurements of multiple nodes
   *  - measurements: a vector of key->pose pair of node measurements
   */
  void addNodeMeasurements(
      const std::vector<std::pair<gtsam::Key, gtsam::Pose3> >& measurements);

  /*! \brief Initialize with new node of a trajectory
   *  - key: Key of first node in new trajectory
   *  - initial_pose: Initial measurement of first node
   *  - add_prior: boolean - add a Prior Factor or not
   */
  void addNewNode(const gtsam::Key& key,
                  const gtsam::Pose3& initial_pose,
                  bool add_prior);

  /*! \brief Add a new between factor to the deformation graph
   *  - key_from: Key of front node to connect between factor
   *  - key_to: Key of back node to connect between factor
   *  - meas: Measurement of between factor
   *  - Estimated position of new node (the back node if node is new)
   */
  void addNewBetween(const gtsam::Key& key_from,
                     const gtsam::Key& key_to,
                     const gtsam::Pose3& meas,
                     const gtsam::Pose3& initial_pose = gtsam::Pose3());

  /*! \brief Add connections from a pose graph node to mesh vertices nodes
   *  - key: Key of pose graph node
   *  - valences: The mesh vertices nodes to connect to
   *  - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   */
  void addNodeValence(const gtsam::Key& key,
                      const Vertices& valences,
                      const char& valence_prefix);

  /*! \brief Remove sll prior factors of nodes that have given prefix
   *  - prefix: prefix of nodes to remove prior
   */
  void removePriorsWithPrefix(const char& prefix);

  /*! \brief Get the optimized estimates for nodes with certain prefix
   *  - prefix: prefix of the nodes to query best estimate
   */
  std::vector<gtsam::Pose3> getOptimizedTrajectory(char prefix) const;

  /*! \brief Deform a mesh based on the deformation graph
   * - original_mesh: mesh to deform
   * - k: how many nearby nodes to use to adjust new position of vertices when
   * - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   * deforming
   */
  pcl::PolygonMesh deformMesh(const pcl::PolygonMesh& original_mesh,
                              const char& prefix,
                              size_t k = 4);

  /*! \brief Deform a mesh based on the deformation graph
   * - original_mesh: mesh to deform
   * - k: how many nearby nodes to use to adjust new position of vertices when
   * - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices deforming
   * - values: values consisting of the key-pose pairs of the optimized mesh
   * vertices
   */
  pcl::PolygonMesh deformMesh(const pcl::PolygonMesh& original_mesh,
                              const char& prefix,
                              const gtsam::Values& values,
                              size_t k = 4);

  /*! \brief Get the number of mesh vertices nodes in the deformation graph
   * - outputs the number of mesh vertices nodes
   */
  inline size_t getNumVertices() const { return vertices_->points.size(); }

  /*! \brief Get the positions of the mesh vertices nodes in the deformation
   * graph
   * - outputs the positions of mesh vertices nodes as a pointcloud
   */
  inline pcl::PointCloud<pcl::PointXYZ> getVertices() const {
    return *vertices_;
  }

  /*! \brief Gets the deformation graph as a graph type (currently the pose
   * graph nodes not included)
   * - outputs graph
   */
  inline Graph getGraph(const char& prefix) const { return graph_.at(prefix); }

  /*! \brief Gets the estimated values since last optimization
   *  - outputs last estimated values as GTSAM Values
   */
  inline gtsam::Values getGtsamValues() const { return values_; }

  /*! \brief Gets the factors added to the backend, minus the detected outliers
   *  - outputs the factors as a GTSAM NonlinearFactorGraph
   */
  inline gtsam::NonlinearFactorGraph getGtsamFactors() const { return nfg_; }

  /*! \brief Gets the pose graph from the backend
   *   - timestamps: map of robot id to sequential timestamps in order to stamp
   * the nodes in the output pose graph msg
   *  - outputs the pose graph in pose_graph_tools PoseGraph type
   */
  inline GraphMsgPtr getPoseGraph(
      const std::map<size_t, std::vector<ros::Time> >& timestamps) {
    return GtsamGraphToRos(nfg_, values_, timestamps);
  }

  /*! \brief Get the consistency factors (ie. the deformation edge factors)
   */
  inline gtsam::NonlinearFactorGraph getConsistencyFactors() const {
    return consistency_factors_;
  }

  /*! \brief Get the intial pose of a keyframe node
   */
  inline gtsam::Pose3 getInitialPose(const char& prefix,
                                     const size_t& index) const {
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

  /*! \brief Never optimize graph, store factors only
   */
  inline void storeOnlyNoOptimization() { do_not_optimize_ = true; }

 private:
  /*! \brief Called within updateMesh to create the DeformationEdgeFactors
   * created from the newly added mesh surfaces
   *  - new_vertices: vertices of the newly added mesh (in updateMesh)
   *  - new_edges: edges created from the sides of the new mesh surfaces
   *  - prefix: the prefixes of the key of the nodes corresponding to mesh
   * vertices
   */
  void updateConsistencyFactors(const Vertices& new_vertices,
                                const std::vector<Edge>& new_edges,
                                const char& prefix);

 private:
  std::map<char, Graph> graph_;
  pcl::PointCloud<pcl::PointXYZ>::Ptr vertices_;
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;

  // Keep track of vertices not part of mesh
  // for embedding trajectory, etc.
  std::map<char, std::vector<gtsam::Pose3> > pg_initial_poses_;

  std::map<char, std::vector<gtsam::Point3> > vertex_positions_;
  // track the prefixes only important in multirobot case
  std::vector<char> vertex_prefixes_;
  // Number of mesh vertices corresponding a particular prefix thus far
  std::map<char, size_t> num_vertices_;

  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;

  // factors
  gtsam::NonlinearFactorGraph nfg_;
  // current estimate
  gtsam::Values values_;

  //// Below separated factor types for debugging
  // factor graph encoding the mesh structure
  gtsam::NonlinearFactorGraph consistency_factors_;
  // factor graph for pose graph related factors
  gtsam::NonlinearFactorGraph pg_factors_;

  // Just store and not optimize
  bool do_not_optimize_;

  // Recalculate only if new measurements added
  bool recalculate_vertices_;
  std::map<char, pcl::PointCloud<pcl::PointXYZRGBA> > last_calculated_vertices_;
};

typedef std::shared_ptr<DeformationGraph> DeformationGraphPtr;
}  // namespace kimera_pgmo