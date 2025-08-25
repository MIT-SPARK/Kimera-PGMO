/**
 * @file   deformation_graph_factor.cpp
 * @brief  Deformation Graph factor
 * @author Yun Chang
 */
#include "kimera_pgmo/deformation_graph_factor.h"

namespace kimera_pgmo {

DeformationEdgeFactor::DeformationEdgeFactor(gtsam::Key node1_key,
                                             gtsam::Key node2_key,
                                             const gtsam::Point3& measurement,
                                             gtsam::SharedNoiseModel model)
    : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model, node1_key, node2_key),
      measurement_(measurement) {}

DeformationEdgeFactor::DeformationEdgeFactor(gtsam::Key node1_key,
                                             gtsam::Key node2_key,
                                             const gtsam::Pose3& node1_pose,
                                             const gtsam::Point3& node2_point,
                                             gtsam::SharedNoiseModel model)
    : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(
          model, node1_key, node2_key) {
  measurement_ =
      node1_pose.rotation().inverse().rotate(node2_point - node1_pose.translation());
}

DeformationEdgeFactor::~DeformationEdgeFactor() = default;

gtsam::Vector DeformationEdgeFactor::evaluateError(const gtsam::Pose3& p1,
                                                   const gtsam::Pose3& p2,
                                                   GtsamJacobianType H1,
                                                   GtsamJacobianType H2) const {
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

gtsam::Point3 DeformationEdgeFactor::measurement() const { return measurement_; }

gtsam::NonlinearFactor::shared_ptr DeformationEdgeFactor::clone() const {
  return gtsam::NonlinearFactor::shared_ptr(new DeformationEdgeFactor(*this));
}

}  // namespace kimera_pgmo
