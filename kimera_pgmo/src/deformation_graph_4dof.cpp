#include "kimera_pgmo/deformation_graph_4dof.h"

#include <gtsam/slam/BetweenFactor.h>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/utils/common_functions.h"
#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {
namespace {

template <typename T, typename Ptr>
const T* cast_to_ptr(const Ptr& ptr) {
  return dynamic_cast<const T*>(ptr.get());
}

}  // namespace

using gtsam::BetweenFactor;
using gtsam::NoiseModelFactor2;
using gtsam::NonlinearFactor;

using GraphValuePair = std::pair<gtsam::NonlinearFactorGraph, gtsam::Values>;

DeformationEdgeFactorToPose4DoF::DeformationEdgeFactorToPose4DoF(
    gtsam::Key node1_key,
    gtsam::Key node2_key,
    const gtsam::Point3& measurement,
    gtsam::SharedNoiseModel model)
    : NoiseModelFactor2<gtsam::Pose3, gtsam::Pose4DoF>(model, node1_key, node2_key),
      measurement_(measurement) {}

DeformationEdgeFactorToPose4DoF::~DeformationEdgeFactorToPose4DoF() = default;

gtsam::Vector DeformationEdgeFactorToPose4DoF::evaluateError(
    const gtsam::Pose3& p1,
    const gtsam::Pose4DoF& p2,
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
    Eigen::MatrixXd Jacobian_2 = Eigen::MatrixXd::Zero(3, 4);
    Jacobian_2 = Jacobian_2 - H_t2;
    *H2 = Jacobian_2;
  }

  return t2_1 - t2_2;
}

NonlinearFactor::shared_ptr DeformationEdgeFactorToPose4DoF::clone() const {
  return NonlinearFactor::shared_ptr(new DeformationEdgeFactorToPose4DoF(*this));
}

DeformationEdgeFactorFromPose4DoF::DeformationEdgeFactorFromPose4DoF(
    gtsam::Key node1_key,
    gtsam::Key node2_key,
    const gtsam::Point3& measurement,
    gtsam::SharedNoiseModel model)
    : NoiseModelFactor2<gtsam::Pose4DoF, gtsam::Pose3>(model, node1_key, node2_key),
      measurement_(measurement) {}

DeformationEdgeFactorFromPose4DoF::~DeformationEdgeFactorFromPose4DoF() = default;

gtsam::Vector DeformationEdgeFactorFromPose4DoF::evaluateError(
    const gtsam::Pose4DoF& p1,
    const gtsam::Pose3& p2,
    GtsamJacobianType H1,
    GtsamJacobianType H2) const {
  // position of node 2 in frame of node 1
  gtsam::Matrix H_t1, H_t2;
  gtsam::Point3 R1z = p1.rotation().rotate(measurement_);
  gtsam::Point3 t1 = p1.translation(H_t1);
  // New position of node 2 according to deformation p1 of node 1
  gtsam::Point3 t2_1 = t1 + R1z;
  gtsam::Point3 t2_2 = p2.translation(H_t2);

  // Calculate Jacobians
  if (H1) {
    Eigen::MatrixXd Jacobian_1 = Eigen::MatrixXd::Zero(3, 4);
    Jacobian_1.block<3, 3>(0, 0) = H_t1;
    Jacobian_1(0, 3) = -R1z(1);
    Jacobian_1(1, 3) = R1z(0);
    *H1 = Jacobian_1;
  }

  if (H2) {
    Eigen::MatrixXd Jacobian_2 = Eigen::MatrixXd::Zero(3, 6);
    Jacobian_2 = Jacobian_2 - H_t2;
    *H2 = Jacobian_2;
  }

  return t2_1 - t2_2;
}

NonlinearFactor::shared_ptr DeformationEdgeFactorFromPose4DoF::clone() const {
  return NonlinearFactor::shared_ptr(new DeformationEdgeFactorFromPose4DoF(*this));
}

gtsam::Values convertPose3ToPose4DoF(const gtsam::Values& values) {
  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_values;

  for (const auto& [key, value] : values) {
    // NOTE (hlim): This conversion strongly assumes that the complement of vertices
    // consists of poses from both robot agents and places.
    // Due to the definition of the deformable point, it should remain as gtsam::Pose3.
    // Others (including place and agent nodes) are converted to gtsam::Pose4DoF.
    if (isVertexKey(key)) {
      new_values.insert(key, value.cast<gtsam::Pose3>());
    } else {
      if (value.dim() == gtsam::Pose3::dimension) {
        // Handle Pose3
        gtsam::Pose4DoF pose4dof(value.cast<gtsam::Pose3>());
        new_values.insert(key, pose4dof);
      } else if (value.dim() == gtsam::Pose4DoF::dimension) {
        // To preserve already-converted factors
        new_values.insert(key, value.cast<gtsam::Pose4DoF>());
      } else {
        throw std::runtime_error("Unsupported value dimension detected.");
      }
    }
  }

  return values;
}

GraphValuePair convertGraphToPose4DoF(const gtsam::NonlinearFactorGraph& original_nfg,
                                      const gtsam::Values& original_values) {
  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_values;

  PGMO_LOG(INFO,
           "Original graph: values={}, factors={}",
           original_values.size(),
           original_nfg.size());

  // This lookup table prevents redundant construction of Pose4DoF objects.
  std::unordered_map<gtsam::Key, gtsam::Pose4DoF> key_to_pose4dof;
  //
  // 1) Convert all Values
  for (const auto& [key, value] : original_values) {
    // NOTE (hlim): This conversion strongly assumes that the complement of vertices
    // consists of poses from both robot agents and places.
    // Due to the definition of the deformable point, it should remain as gtsam::Pose3.
    // Others (including place and agent nodes) are converted to gtsam::Pose4DoF.
    if (isVertexKey(key)) {
      new_values.insert(key, value.cast<gtsam::Pose3>());
    } else {
      if (value.dim() == gtsam::Pose3::dimension) {
        // Handle Pose3
        const gtsam::Pose4DoF& pose4dof(value.cast<gtsam::Pose3>());
        new_values.insert(key, pose4dof);
        key_to_pose4dof[key] = pose4dof;
      } else if (value.dim() == gtsam::Pose4DoF::dimension) {
        // To preserve already-converted factors
        const gtsam::Pose4DoF& pose4dof = value.cast<gtsam::Pose4DoF>();
        new_values.insert(key, pose4dof);
        key_to_pose4dof[key] = pose4dof;
      } else {
        throw std::runtime_error("Unsupported value dimension detected.");
      }
    }
  }

  // 2) Convert all factors into the Pose4DoF format
  for (const auto& factor : original_nfg) {
    bool conversion_succeeded = false;
    const auto pose3_prior = cast_to_ptr<gtsam::PriorFactor<gtsam::Pose3>>(factor);
    if (pose3_prior) {
      conversion_succeeded = true;

      const auto sigmas = pose3_prior->noiseModel()->sigmas();
      auto sigma_vector =
          (gtsam::Vector(4) << sigmas(3), sigmas(4), sigmas(5), sigmas(0)).finished();
      const gtsam::SharedNoiseModel& noise =
          gtsam::noiseModel::Diagonal::Variances(sigma_vector);

      const auto prior_key = factor->keys().at(0);
      if (isVertexKey(prior_key)) {
        new_graph.add(*pose3_prior);
      } else {
        const auto& pose_4dof = gtsam::Pose4DoF(pose3_prior->prior());
        new_graph.add(gtsam::PriorFactor<gtsam::Pose4DoF>(prior_key, pose_4dof, noise));
      }

      continue;
    }

    gtsam::Key key1 = factor->keys().at(0);
    gtsam::Key key2 = factor->keys().at(1);
    const auto pose3_between = cast_to_ptr<BetweenFactor<gtsam::Pose3>>(factor);
    if (pose3_between) {
      conversion_succeeded = true;

      const auto sigmas = pose3_between->noiseModel()->sigmas();
      auto sigma_vector =
          (gtsam::Vector(4) << sigmas(3), sigmas(4), sigmas(5), sigmas(0)).finished();
      const gtsam::SharedNoiseModel& noise =
          gtsam::noiseModel::Diagonal::Variances(sigma_vector);

      // NOTE(hlim): This procedure is crucial for 4DoF optimization.
      // Do not directly use `pose3_between->measured()` as input!
      const gtsam::Pose3& pose_from = original_values.at<gtsam::Pose3>(key1);
      gtsam::Pose3 pose_to = pose_from.compose(pose3_between->measured());
      const auto& pose_from_4dof = key_to_pose4dof.at(key1);
      gtsam::Pose4DoF pose_to_4dof(std::move(pose_to));
      new_graph.add(BetweenFactor<gtsam::Pose4DoF>(
          key1, key2, pose_from_4dof.between(pose_to_4dof), noise));
      continue;
    }

    bool is_key1_in_vertex_prefix = isVertexKey(key1);
    bool is_key2_in_vertex_prefix = isVertexKey(key2);
    const auto deformation_factor = cast_to_ptr<DeformationEdgeFactor>(factor);
    if (deformation_factor) {
      conversion_succeeded = true;
      // Case 1: Both keys correspond to deformable vertices
      if (is_key1_in_vertex_prefix && is_key2_in_vertex_prefix) {
        new_graph.add(factor);
        // Case 2: From a deformable vertex to an agent (or place)
      } else if (is_key1_in_vertex_prefix) {
        new_graph.add(
            DeformationEdgeFactorToPose4DoF(key1,
                                            key2,
                                            deformation_factor->measurement(),
                                            deformation_factor->noiseModel()));

        // Case 3: From an agent (or place) to a deformable vertex
      } else if (is_key2_in_vertex_prefix) {
        new_graph.add(
            DeformationEdgeFactorFromPose4DoF(key1,
                                              key2,
                                              deformation_factor->measurement(),
                                              deformation_factor->noiseModel()));
      }

      continue;
    }

    if (cast_to_ptr<BetweenFactor<gtsam::Pose4DoF>>(factor)) {
      throw std::runtime_error(
          "Error: A converted factor was attempted to be converted again "
          "(BetweenFactor<gtsam::Pose4DoF>).");
    }

    if (cast_to_ptr<DeformationEdgeFactorToPose4DoF>(factor)) {
      throw std::runtime_error(
          "Error: A converted factor was attempted to be converted again "
          "(DeformationEdgeFactorToPose4DoF).");
    }

    if (cast_to_ptr<DeformationEdgeFactorFromPose4DoF>(factor)) {
      throw std::runtime_error(
          "Error: A converted factor was attempted to be converted again "
          "(DeformationEdgeFactorFromPose4DoF).");
    }

    if (!conversion_succeeded) {
      PGMO_LOG(WARN, "Factor was not converted");
      factor->print();
    }
  }

  PGMO_LOG(INFO,
           "Converted graph: values={}, factors={}",
           new_values.size(),
           new_graph.size());

  if (original_values.size() != new_values.size()) {
    PGMO_LOG(WARN,
             "Conversion changed value count: {} -> {}",
             original_values.size(),
             new_values.size());
  }

  if (original_nfg.size() != new_graph.size()) {
    PGMO_LOG(WARN,
             "Conversion changed factor count: {} -> {}",
             original_nfg.size(),
             new_graph.size());
  }

  return {new_graph, new_values};
}

gtsam::Values convertPose4DoFToPose3(const gtsam::Values& values) {
  gtsam::Values result;
  for (const auto& [key, value] : values) {
    const auto pose = dynamic_cast<const gtsam::GenericValue<gtsam::Pose4DoF>*>(&value);
    if (pose) {
      const auto& pose_4dof = pose->value();
      result.insert(key, gtsam::Pose3(pose_4dof.rotation(), pose_4dof.translation()));
    } else {
      result.insert(key, value);
    }
  }

  return result;
}

}  // namespace kimera_pgmo
