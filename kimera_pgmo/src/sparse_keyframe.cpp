/**
 * @file   kimera_pgmo_interface.cpp
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#include "kimera_pgmo/sparse_keyframe.h"

#include "kimera_pgmo/utils/common_functions.h"
#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {

using pose_graph_tools::PoseGraph;
using pose_graph_tools::PoseGraphEdge;

void SparseKeyframe::initialize(const gtsam::Key& sparse_key,
                                size_t robot_id,
                                size_t pose_id) {
  const gtsam::Symbol key_symb(GetRobotPrefix(robot_id), pose_id);
  key = sparse_key;
  keyed_transforms.insert({key_symb, gtsam::Pose3()});
}

bool SparseKeyframe::addNewEdge(const PoseGraphEdge& new_edge,
                                double trans_threshold,
                                double rot_threshold) {
  if (!active) {
    SPARK_LOG(ERROR) << "Cannot add edge to inactive sparse key-frame.";
    return false;
  }

  if (new_edge.type != PoseGraphEdge::ODOM) {
    SPARK_LOG(ERROR) << "Trying to add non-ODOM edge to sparse key-frame.";
    return false;
  }

  const gtsam::Symbol from_key(GetRobotPrefix(new_edge.robot_from), new_edge.key_from);
  const gtsam::Symbol to_key(GetRobotPrefix(new_edge.robot_to), new_edge.key_to);

  if (keyed_transforms.rbegin()->first != to_key - 1) {
    SPARK_LOG(ERROR)
        << "Attempting to insert non-consecutive nodes to sparse key-frame";
    return false;
  }

  const gtsam::Pose3 measure(new_edge.pose.matrix());
  current_transform = current_transform.compose(measure);
  edges.push_back(new_edge);

  double dist_total = current_transform.translation().norm();
  double rot_total = current_transform.rotation().xyz().norm();
  if (dist_total > trans_threshold || rot_total > rot_threshold) {
    active = false;
    return false;
  }

  keyed_transforms.insert({to_key, current_transform});
  return true;
}

}  // namespace kimera_pgmo
