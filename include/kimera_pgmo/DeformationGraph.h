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
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/CommonStructs.h"

namespace kimera_pgmo {

// Define a factor type for edges of deformation graph
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
};

class DeformationGraph {
 public:
  DeformationGraph();
  ~DeformationGraph();

  bool Initialize(double pgo_trans_threshold, double pgo_rot_threshold);

  void update();

  inline void updateMesh(const pcl::PolygonMesh& mesh) {
    mesh_structure_ = mesh;
  }

  void addMeasurement(const Vertex& v, const geometry_msgs::Pose& transform);

  void addNodeMeasurement(const size_t& node_number,
                          const gtsam::Pose3 delta_pose);

  void initFirstNode(const gtsam::Pose3& initial_pose);

  void addNewBetween(const size_t& from,
                     const size_t& to,
                     const gtsam::Pose3& meas,
                     const gtsam::Pose3& initial_pose = gtsam::Pose3());

  void addNode(const pcl::PointXYZ& position,
               Vertices valences,
               bool connect_to_previous = false);

  void updateNodeValence(size_t i,
                         Vertices valences,
                         bool connect_to_previous = false);

  std::vector<gtsam::Pose3> getOptimizedTrajectory() const;

  pcl::PolygonMesh deformMesh(const pcl::PolygonMesh& original_mesh,
                              size_t k = 4);

  inline size_t getNumVertices() const { return vertices_.points.size(); }
  inline pcl::PointCloud<pcl::PointXYZ> getVertices() const {
    return vertices_;
  }
  inline Graph getGraph() const { return graph_; }

  inline GraphMsgPtr getPoseGraph(const std::vector<ros::Time>& timestamps) {
    return GtsamGraphToRos(nfg_, values_, timestamps);
  }

 private:
  void updateConsistencyFactors(const Graph& new_graph);

  void addNonMeshNodesToGraph(Graph* graph);

 private:
  Graph graph_;
  pcl::PointCloud<pcl::PointXYZ> vertices_;
  // Keep track of vertices not part of mesh
  // for embedding trajectory, etc.
  std::vector<pcl::PointXYZ> pg_vertices_;
  std::vector<gtsam::Pose3> pg_initial_poses_;
  std::vector<Vertices> pg_connections_;

  std::map<Vertex, gtsam::Point3> vertex_positions_;

  // track mesh if one used to create deformation graph
  pcl::PolygonMesh mesh_structure_;

  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;

  // factors
  gtsam::NonlinearFactorGraph nfg_;
  // current estimate
  gtsam::Values values_;

  //// Below separated factor types for debugging
  // factor graph encoding the mesh structure
  gtsam::NonlinearFactorGraph consistency_factors_;
  // factor graph storing the prior factors for distortions
  gtsam::NonlinearFactorGraph prior_factors_;
  // factor graph for pose graph related factors
  gtsam::NonlinearFactorGraph pg_factors_;

  // Recalculate only if new measurements added
  bool recalculate_vertices_;
  pcl::PointCloud<pcl::PointXYZRGBA> last_calculated_vertices_;
};
}  // namespace kimera_pgmo