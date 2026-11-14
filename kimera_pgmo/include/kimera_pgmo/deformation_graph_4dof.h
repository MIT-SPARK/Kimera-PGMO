#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <kimera_rpgo/utils/pose_4dof.h>

namespace kimera_pgmo {

#if GTSAM_VERSION_MAJOR <= 4 && GTSAM_VERSION_MINOR < 3
using GtsamJacobianType = boost::optional<gtsam::Matrix&>;
#define JACOBIAN_DEFAULT \
  {}
#else
using GtsamJacobianType = gtsam::OptionalMatrixType;
#define JACOBIAN_DEFAULT nullptr
#endif

/*!
 * \class DeformationEdgeFactorToPose4DoF
 * \brief Factor that constrains a 3D pose/mesh point (Pose3) and a 4DoF pose (Pose4DoF)
 * for the deformation graph representation.
 */
class DeformationEdgeFactorToPose4DoF
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose4DoF> {
 private:
  gtsam::Point3 measurement_;

 public:
  DeformationEdgeFactorToPose4DoF(gtsam::Key node1_key,
                                  gtsam::Key node2_key,
                                  const gtsam::Point3& measurement,
                                  gtsam::SharedNoiseModel model);

  virtual ~DeformationEdgeFactorToPose4DoF();

  gtsam::Vector evaluateError(const gtsam::Pose3& p1,
                              const gtsam::Pose4DoF& p2,
                              GtsamJacobianType H1 = JACOBIAN_DEFAULT,
                              GtsamJacobianType H2 = JACOBIAN_DEFAULT) const override;

  gtsam::Point3 measurement() const { return measurement_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

/*!
 * \class DeformationEdgeFactorFromPose4DoF
 * \brief Factor that constrains a 4DoF pose (Pose4DoF) and a 3D pose/mesh point (Pose3)
 * for the deformation graph representation.
 */
class DeformationEdgeFactorFromPose4DoF
    : public gtsam::NoiseModelFactor2<gtsam::Pose4DoF, gtsam::Pose3> {
 private:
  gtsam::Point3 measurement_;

 public:
  DeformationEdgeFactorFromPose4DoF(gtsam::Key node1_key,
                                    gtsam::Key node2_key,
                                    const gtsam::Point3& measurement,
                                    gtsam::SharedNoiseModel model);

  virtual ~DeformationEdgeFactorFromPose4DoF();

  gtsam::Vector evaluateError(const gtsam::Pose4DoF& p1,
                              const gtsam::Pose3& p2,
                              GtsamJacobianType H1 = JACOBIAN_DEFAULT,
                              GtsamJacobianType H2 = JACOBIAN_DEFAULT) const override;

  gtsam::Point3 measurement() const { return measurement_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override;
};

#undef JACOBIAN_DEFAULT

/*! \brief Convert a gtsam::Pose3 to a gtsam::Pose4DoF representation
 *  - values: the original estimated values of the pose graph
 *  \return Factor graph values converted to gtsam::Pose4DoF selectively
 *  Note, mesh points should remain as gtsam::Pose3
 */
gtsam::Values convertPose3ToPose4DoF(const gtsam::Values& values);

/*! \brief Convert a pose graph to a 4 degrees-of-freedom (DoF) representation
 *  - original_nfg: the original nonlinear factor graph
 *  - original_values: the original estimated values of the pose graph
 *  \return a pair containing the converted nonlinear factor graph and values
 */
std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> convertGraphToPose4DoF(
    const gtsam::NonlinearFactorGraph& original_nfg,
    const gtsam::Values& original_values);

/*! \brief Convert a 4 degrees-of-freedom (DoF) pose estimation back to full
 *         6 DoF pose representation
 *  - values: the values in the 4 DoF representation
 *  \return the converted values in full 6 DoF pose representation
 */
gtsam::Values convertPose4DoFToPose3(const gtsam::Values& values);

}  // namespace kimera_pgmo
