/**
 * @file   kimera_pgmo_interface.h
 * @brief  KimeraPgmo interface class: base class and methods for Kimera PGMO
 * @author Yun Chang
 */
#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <pose_graph_tools/pose_graph.h>

namespace kimera_pgmo {

struct SparseKeyframe {
  gtsam::Key key;
  std::map<gtsam::Key, gtsam::Pose3> keyed_transforms;
  bool active = true;
  gtsam::Pose3 current_transform;
  std::vector<pose_graph_tools::PoseGraphEdge> edges;

  void initialize(const gtsam::Key& sparse_key, size_t robot_id, size_t pose_id);

  bool addNewEdge(const pose_graph_tools::PoseGraphEdge& new_edge,
                  double trans_threshold,
                  double rot_threshold);
};

}  // namespace kimera_pgmo
