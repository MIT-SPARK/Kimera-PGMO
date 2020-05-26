/**
 * @file   DeformationGraph.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */

#include <map>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <geometry_msgs/Pose.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mesher_mapper/CommonStructs.h"

namespace mesher_mapper {

// Define a factor type for edges of deformation graph
class DeformationEdgeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  gtsam::Point3 node1_position;
  gtsam::Point3 node2_position;

 public:
  DeformationEdgeFactor(gtsam::Key node1_key,
                        gtsam::Key node2_key,
                        const gtsam::Point3& node1_point,
                        const gtsam::Point3& node2_point,
                        gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model,
                                                             node1_key,
                                                             node2_key),
        node1_position(node1_point),
        node2_position(node2_point) {}
  ~DeformationEdgeFactor() {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& p1,
      const gtsam::Pose3& p2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {
    gtsam::Matrix H_R1, H_t1, H_t2;
    gtsam::Rot3 R1 = p1.rotation();
    gtsam::Point3 t1 = p1.translation(H_t1);
    // New position of node 2 according to deformation p1 of node 1
    gtsam::Point3 t2_1 =
        t1 + node1_position + R1.rotate(node2_position - node1_position, H_R1);
    gtsam::Point3 t2 = p2.translation(H_t2);
    // New position of node 2 according to deformation p2 of node 2
    gtsam::Point3 t2_2 = t2 + node2_position;

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

  void createFromMesh(const pcl::PolygonMesh& mesh);

  void addMesh(const pcl::PolygonMesh& mesh);

  void addRelativeMeasurement(const Vertex& v1,
                              const Vertex& v2,
                              const geometry_msgs::Pose& transform);

  void optimize();

  pcl::PolygonMesh deformMesh(const pcl::PolygonMesh& original_mesh) const;

  inline size_t getNumVertices() const { return vertices_.points.size(); }
  inline pcl::PointCloud<pcl::PointXYZ> getVertices() const {
    return vertices_;
  }

 private:
  Graph graph_;
  pcl::PointCloud<pcl::PointXYZ> vertices_;

  // factor graph encoding the mesh structure
  gtsam::NonlinearFactorGraph consistency_factors_;
  // factor graph storing the between factors for the loop closures
  gtsam::NonlinearFactorGraph relative_transforms_;
  // current estimate
  gtsam::Values values_;
};
}  // namespace mesher_mapper