
/**
 * @file   deformation_edge_factor.h
 * @brief  Deformation Graph factor
 * @author Yun Chang
 */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <kimera_rpgo/utils/pose_4dof.h>

#include <type_traits>

namespace kimera_pgmo {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using GtsamJacobianType = boost::optional<gtsam::Matrix&>;
#define JACOBIAN_DEFAULT \
  {}
#else
using GtsamJacobianType = gtsam::OptionalMatrixType;
#define JACOBIAN_DEFAULT nullptr
#endif

/*! \brief Constrain two deformation graph nodes to preserve their original offset.
 *
 * The measurement z is node 2's original position expressed in node 1's original
 * frame: z = R1_original^T (t2_original - t1_original).
 * For current estimates p1 = (R1, t1) and p2 = (R2, t2), node 1 predicts node 2's
 * position as t2_1 = t1 + R1 z. Node 2 places itself at t2_2 = t2. The residual
 * is their difference, t2_1 - t2_2, in the world frame. R2 does not enter it.
 * Either endpoint may be Pose3 or Pose4DoF; the residual is always 3D.
 */
template <typename From, typename To>
class DeformationEdgeFactorT : public gtsam::NoiseModelFactor2<From, To> {
 private:
  gtsam::Point3 measurement_;

 public:
  DeformationEdgeFactorT(gtsam::Key node1_key,
                         gtsam::Key node2_key,
                         const gtsam::Point3& measurement,
                         gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<From, To>(model, node1_key, node2_key),
        measurement_(measurement) {}

  DeformationEdgeFactorT(gtsam::Key node1_key,
                         gtsam::Key node2_key,
                         const gtsam::Pose3& node1_pose,
                         const gtsam::Point3& node2_point,
                         gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<From, To>(model, node1_key, node2_key) {
    measurement_ =
        node1_pose.rotation().inverse().rotate(node2_point - node1_pose.translation());
  }

  virtual ~DeformationEdgeFactorT() {}

  gtsam::Vector evaluateError(const From& p1,
                              const To& p2,
                              GtsamJacobianType H1 = JACOBIAN_DEFAULT,
                              GtsamJacobianType H2 = JACOBIAN_DEFAULT) const override {
    gtsam::Matrix H_t1, H_t2;
    const auto R1 = p1.rotation();
    const auto t1 = p1.translation(H_t1);
    const auto R1z = R1.rotate(measurement_);
    // Node 2's predicted position under node 1's deformation, versus its own.
    const gtsam::Point3 t2_1 = t1 + R1z;
    const auto t2_2 = p2.translation(H_t2);

    // Differentiate with respect to each pose's local tangent coordinates.
    if (H1) {
      gtsam::Matrix Jacobian_1 = H_t1;
      if constexpr (std::is_same_v<From, gtsam::Pose4DoF>) {
        // Pose4DoF: [translation, yaw], with fixed roll and pitch.
        // d(R1 z)/d(yaw) = e_z x (R1 z), about the world vertical axis.
        Jacobian_1.col(3) += gtsam::Vector3(-R1z.y(), R1z.x(), 0.0);
      } else {
        // Pose3: [rotation, translation]; H_R1 = -R1 [z]_x.
        gtsam::Matrix33 H_R1;
        R1.rotate(measurement_, H_R1);
        Jacobian_1.block<3, 3>(0, 0) += H_R1;
      }

      *H1 = Jacobian_1;
    }

    if (H2) {
      // Only node 2's translation enters the residual, with a negative sign.
      *H2 = -H_t2;
    }

    return t2_1 - t2_2;
  }

  inline gtsam::Point3 measurement() const { return measurement_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(new DeformationEdgeFactorT(*this));
  }
};

using DeformationEdgeFactor = DeformationEdgeFactorT<gtsam::Pose3, gtsam::Pose3>;
using DeformationEdgeFactorToPose4DoF =
    DeformationEdgeFactorT<gtsam::Pose3, gtsam::Pose4DoF>;
using DeformationEdgeFactorFromPose4DoF =
    DeformationEdgeFactorT<gtsam::Pose4DoF, gtsam::Pose3>;
using DeformationEdgeFactor4DoF =
    DeformationEdgeFactorT<gtsam::Pose4DoF, gtsam::Pose4DoF>;

#undef JACOBIAN_DEFAULT

}  // namespace kimera_pgmo
