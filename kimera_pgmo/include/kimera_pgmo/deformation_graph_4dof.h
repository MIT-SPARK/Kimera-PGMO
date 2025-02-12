#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/slam/BetweenFactor.h>
#include <kimera_rpgo/utils/pose_4dof.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pose_graph_tools/pose_graph.h>

#include "kimera_pgmo/deformation_graph.h"
#include "utils/logging.h"

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
                                  gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose4DoF>(model,
                                                                node1_key,
                                                                node2_key),
        measurement_(measurement) {}

  virtual ~DeformationEdgeFactorToPose4DoF() {}

  gtsam::Vector evaluateError(const gtsam::Pose3& p1,
                              const gtsam::Pose4DoF& p2,
                              GtsamJacobianType H1 = JACOBIAN_DEFAULT,
                              GtsamJacobianType H2 = JACOBIAN_DEFAULT) const override {
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

  inline gtsam::Point3 measurement() const { return measurement_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new DeformationEdgeFactorToPose4DoF(*this));
  }
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
                                    gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose4DoF, gtsam::Pose3>(model,
                                                                node1_key,
                                                                node2_key),
        measurement_(measurement) {}

  virtual ~DeformationEdgeFactorFromPose4DoF() {}

  gtsam::Vector evaluateError(const gtsam::Pose4DoF& p1,
                              const gtsam::Pose3& p2,
                              GtsamJacobianType H1 = JACOBIAN_DEFAULT,
                              GtsamJacobianType H2 = JACOBIAN_DEFAULT) const override {
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

  inline gtsam::Point3 measurement() const { return measurement_; }

  gtsam::NonlinearFactor::shared_ptr clone() const override {
    return gtsam::NonlinearFactor::shared_ptr(
        new DeformationEdgeFactorFromPose4DoF(*this));
  }
};

#undef JACOBIAN_DEFAULT

/*! \brief Check if a given prefix exists in the id-to-prefix map.
 *
 *  - id_to_prefix_map: A map where the key is an ID and the value is a character
 * prefix.
 *  - prefix: The prefix character to check for in the map.
 *  \return True if the prefix exists in the map, false otherwise.
 */
inline bool prefixExistsInMap(const std::map<size_t, char>& id_to_prefix_map,
                              const char prefix) {
  return std::any_of(id_to_prefix_map.begin(),
                     id_to_prefix_map.end(),
                     [prefix](const auto& pair) { return pair.second == prefix; });
}

/*! \brief Check if the first character of a given string exists as a prefix in the map.
 *
 *  - id_to_prefix_map: A map where the key is an ID and the value is a character
 * prefix.
 *  - readable_key: A string whose first character will be checked in the map.
 *  \return True if the first character of readable_key exists in the map, false
 * otherwise.
 *
 *  \note If the readable_key is empty, the function returns false.
 */
inline bool prefixExistsInMap(const std::map<size_t, char>& id_to_prefix_map,
                              const std::string& readable_key) {
  if (readable_key.empty()) return false;
  const char prefix = readable_key[0];
  return prefixExistsInMap(id_to_prefix_map, prefix);
}

/*! \brief Check if the first character of a formatted gtsam::Key exists as a prefix in
 * the map.
 *
 *  - id_to_prefix_map: A map where the key is an ID and the value is a character
 * prefix.
 *  - key: A gtsam::Key whose formatted representation will be checked.
 *  \return True if the first character of the formatted key exists in the map, false
 * otherwise.
 */
inline bool prefixExistsInMap(const std::map<size_t, char>& id_to_prefix_map,
                              const gtsam::Key key) {
  const char prefix = gtsam::DefaultKeyFormatter(key)[0];
  return prefixExistsInMap(id_to_prefix_map, prefix);
}

/*! \brief Convert a gtsam::Pose3 to a gtsam::Pose4DoF representation
 *  - values: the original estimated values of the pose graph
 *  \return Factor graph values converted to gtsam::Pose4DoF selectively
 *  Note, mesh points should remain as gtsam::Pose3
 */
inline gtsam::Values convertPose3ToPose4DoF(const gtsam::Values& values) {
  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_values;

  for (const auto& key_value : values) {
    const gtsam::Key key = key_value.key;
    bool is_in_vertex_prefix = prefixExistsInMap(robot_id_to_vertex_prefix, key);

    // NOTE (hlim): This conversion strongly assumes that the complement of vertices
    // consists of poses from both robot agents and places.
    // Due to the definition of the deformable point, it should remain as gtsam::Pose3.
    // Others (including place and agent nodes) are converted to gtsam::Pose4DoF.
    if (is_in_vertex_prefix) {
      new_values.insert(key, key_value.value.cast<gtsam::Pose3>());
    } else {
      if (key_value.value.dim() == gtsam::Pose3::dimension) {
        // Handle Pose3
        gtsam::Pose4DoF pose4dof(key_value.value.cast<gtsam::Pose3>());
        new_values.insert(key, pose4dof);

      } else if (key_value.value.dim() == gtsam::Pose4DoF::dimension) {
        // To preserve already-converted factors
        new_values.insert(key, key_value.value.cast<gtsam::Pose4DoF>());
      } else {
        throw std::runtime_error("Unsupported value dimension detected.");
      }
    }
  }

  return values;
};

/*! \brief Convert a pose graph to a 4 degrees-of-freedom (DoF) representation
 *  - original_nfg: the original nonlinear factor graph
 *  - original_values: the original estimated values of the pose graph
 *  \return a pair containing the converted nonlinear factor graph and values
 */
inline std::pair<gtsam::NonlinearFactorGraph, gtsam::Values> convertGraphToPose4DoF(
    const gtsam::NonlinearFactorGraph& original_nfg,
    const gtsam::Values& original_values) {
  gtsam::NonlinearFactorGraph new_graph;
  gtsam::Values new_values;

  SPARK_LOG(INFO) << "# of original values:" << original_values.size()
                  << ", # of original factors: " << original_nfg.size();

  // This lookup table prevents redundant construction of Pose4DoF objects.
  std::unordered_map<gtsam::Key, gtsam::Pose4DoF> key_to_pose4dof;
  //
  // 1) Convert all Values
  for (const auto& key_value : original_values) {
    const gtsam::Key key = key_value.key;
    bool is_in_vertex_prefix = prefixExistsInMap(robot_id_to_vertex_prefix, key);

    // NOTE (hlim): This conversion strongly assumes that the complement of vertices
    // consists of poses from both robot agents and places.
    // Due to the definition of the deformable point, it should remain as gtsam::Pose3.
    // Others (including place and agent nodes) are converted to gtsam::Pose4DoF.
    if (is_in_vertex_prefix) {
      new_values.insert(key, key_value.value.cast<gtsam::Pose3>());
    } else {
      if (key_value.value.dim() == gtsam::Pose3::dimension) {
        // Handle Pose3
        const gtsam::Pose4DoF& pose4dof(key_value.value.cast<gtsam::Pose3>());
        new_values.insert(key, pose4dof);
        key_to_pose4dof[key] = pose4dof;
      } else if (key_value.value.dim() == gtsam::Pose4DoF::dimension) {
        // To preserve already-converted factors
        const gtsam::Pose4DoF& pose4dof = key_value.value.cast<gtsam::Pose4DoF>();
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

    auto pose3_prior =
        boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor);
    if (pose3_prior) {
      conversion_succeeded = true;

      const auto sigmas = pose3_prior->noiseModel()->sigmas();
      auto sigma_vector =
          (gtsam::Vector(4) << sigmas(3), sigmas(4), sigmas(5), sigmas(0)).finished();
      const gtsam::SharedNoiseModel& noise =
          gtsam::noiseModel::Diagonal::Variances(sigma_vector);

      const auto& prior_key = factor->keys().at(0);
      bool is_in_vertex_prefix =
          prefixExistsInMap(robot_id_to_vertex_prefix, prior_key);

      if (is_in_vertex_prefix) {
        new_graph.add(*pose3_prior);
      } else {
        const auto& pose_4dof = gtsam::Pose4DoF(pose3_prior->prior());
        new_graph.add(gtsam::PriorFactor<gtsam::Pose4DoF>(prior_key, pose_4dof, noise));
      }
      continue;
    }

    gtsam::Key key1 = factor->keys().at(0);
    gtsam::Key key2 = factor->keys().at(1);

    auto pose3_between =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
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
      new_graph.add(gtsam::BetweenFactor<gtsam::Pose4DoF>(
          key1, key2, pose_from_4dof.between(pose_to_4dof), noise));
      continue;
    }
    bool is_key1_in_vertex_prefix = prefixExistsInMap(robot_id_to_vertex_prefix, key1);
    bool is_key2_in_vertex_prefix = prefixExistsInMap(robot_id_to_vertex_prefix, key2);

    auto deformation_factor =
        boost::dynamic_pointer_cast<DeformationEdgeFactor>(factor);
    if (deformation_factor) {
      conversion_succeeded = true;
      // Case 1: Both keys correspond to deformable vertices
      if (is_key1_in_vertex_prefix && is_key2_in_vertex_prefix) {
        new_graph.add(deformation_factor);
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

    auto pose4dof_between =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose4DoF>>(factor);
    if (pose4dof_between) {
      throw std::runtime_error(
          "Error: A converted factor was attempted to be converted again "
          "(BetweenFactor<gtsam::Pose4DoF>).");
    }

    auto deformation_factor_to_pose4dof =
        boost::dynamic_pointer_cast<DeformationEdgeFactorToPose4DoF>(factor);
    if (deformation_factor_to_pose4dof) {
      throw std::runtime_error(
          "Error: A converted factor was attempted to be converted again "
          "(DeformationEdgeFactorToPose4DoF).");
    }

    auto deformation_factor_from_pose4dof =
        boost::dynamic_pointer_cast<DeformationEdgeFactorFromPose4DoF>(factor);
    if (deformation_factor_from_pose4dof) {
      throw std::runtime_error(
          "Error: A converted factor was attempted to be converted again "
          "(DeformationEdgeFactorFromPose4DoF).");
    }

    if (!conversion_succeeded) {
      SPARK_LOG(WARNING) << "Unexpected conversion happens!";
      factor->print();
    }
  }

  SPARK_LOG(INFO) << "# of converted values:" << new_values.size()
                  << ", # of converted factors:" << new_graph.size();

  if (original_values.size() != new_values.size()) {
    SPARK_LOG(WARNING) << "Size mismatch in values after conversion: "
                       << "original_values.size() = " << original_values.size()
                       << ", new_values.size() = " << new_values.size();
  }

  if (original_nfg.size() != new_graph.size()) {
    SPARK_LOG(WARNING) << "Size mismatch in factor graph after conversion: "
                       << "original_nfg.size() = " << original_nfg.size()
                       << ", new_graph.size() = " << new_graph.size();
  }
  return {new_graph, new_values};
};

/*! \brief Convert a 4 degrees-of-freedom (DoF) pose estimation back to full
 *         6 DoF pose representation
 *  - values: the values in the 4 DoF representation
 *  \return the converted values in full 6 DoF pose representation
 */
inline gtsam::Values convertPose4DoFToPose3(const gtsam::Values& values) {
  gtsam::Values new_values;

  for (const auto& key_value : values) {
    const gtsam::Key key = key_value.key;
    bool is_in_vertex_prefix = prefixExistsInMap(robot_id_to_vertex_prefix, key);

    // NOTE(hlim): Currently, assume that we convert
    // deformation vertices are gtsam::Pose3,
    // and others (including place and agent nodes) are gtsam::Pose4DoF
    if (is_in_vertex_prefix) {
      new_values.insert(key, values.at<gtsam::Pose3>(key));
    } else {
      const gtsam::Pose4DoF& updated_pose = values.at<gtsam::Pose4DoF>(key);
      gtsam::Pose3 pose3(updated_pose.rotation(), updated_pose.translation());
      new_values.insert(key, pose3);
    }
  }

  return new_values;
};

}  // namespace kimera_pgmo
