/**
 * @file   deformation_graph_factor.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */
#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>

namespace kimera_pgmo {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using GtsamJacobianType = boost::optional<gtsam::Matrix&>;
#define JACOBIAN_DEFAULT \
  {}
#else
using GtsamJacobianType = gtsam::OptionalMatrixType;
#define JACOBIAN_DEFAULT nullptr
#endif

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
                        gtsam::SharedNoiseModel model);

  DeformationEdgeFactor(gtsam::Key node1_key,
                        gtsam::Key node2_key,
                        const gtsam::Pose3& node1_pose,
                        const gtsam::Point3& node2_point,
                        gtsam::SharedNoiseModel model);

  virtual ~DeformationEdgeFactor();

  gtsam::Vector evaluateError(const gtsam::Pose3& p1,
                              const gtsam::Pose3& p2,
                              GtsamJacobianType H1 = JACOBIAN_DEFAULT,
                              GtsamJacobianType H2 = JACOBIAN_DEFAULT) const override;
  gtsam::Point3 measurement() const;
  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

#undef JACOBIAN_DEFAULT

}  // namespace kimera_pgmo
