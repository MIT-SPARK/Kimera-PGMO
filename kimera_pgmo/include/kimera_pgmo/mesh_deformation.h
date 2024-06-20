/**
 * @file   mesh_deformation.h
 * @brief  Functions for deforming mesh vertices
 * @author Yun Chang
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo/utils/common_structs.h"
#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {
namespace deformation {

class SearchTree {
 public:
  explicit SearchTree(double resolution = 1.0);

  ~SearchTree();

  size_t getLeafCount() const;

  void addPoint(const gtsam::Point3& point, bool valid);

  void removePoint(size_t index);

  void search(const traits::Pos& point,
              size_t k,
              std::vector<int>& nn_index,
              std::vector<float>& nn_sq_dist) const;

 private:
  struct Impl;
  std::unique_ptr<Impl> impl_;
};

// Calculate new point location from k points
traits::Pos interpPoint(std::set<size_t>& control_points_seen,
                        char prefix,
                        const std::vector<gtsam::Point3>& control_points,
                        const gtsam::Values& values,
                        const SearchTree& octree,
                        size_t k,
                        const traits::Pos& vi);

/*! \brief Deform a points (i.e. the vertices of a mesh) based on the
 * controls points via deformation
 * - original_points: set of points to deform
 * - prefix: a char to distinguish the type of control points
 * - control_points: original positions of the control points. In the case of
 * mesh vertices, these are the original positions of the simplified mesh.
 * - values: key-value pairs. Where each key should be gtsam::Symbol(prefix,
 * idx-in-control-points) from the previous two arguments.
 * - k: how many nearby nodes to use to adjust new position of vertices
 */
template <typename CloudOut,
          typename CloudIn,
          std::enable_if_t<!traits::has_get_stamp<CloudIn>::value, bool> = true>
void deformPoints(CloudOut& new_points,
                  std::vector<std::set<size_t>>& control_point_map,
                  const CloudIn& points,
                  char prefix,
                  const std::vector<gtsam::Point3>& control_points,
                  const std::vector<Timestamp>& /* control_point_stamps */,
                  const gtsam::Values& values,
                  size_t k = 4,
                  double /* tol_t */ = 10.0,
                  const std::vector<size_t>* indices = nullptr) {
  // Check if there are points to deform
  const size_t num_points = indices ? indices->size() : traits::num_vertices(points);
  if (!num_points) {
    return;
  }

  // Cannot deform if no nodes in the deformation graph
  if (control_points.size() == 0) {
    SPARK_LOG(WARNING) << "No control points. No deformation";
    return;
  }

  control_point_map.clear();

  // Build Octree
  SearchTree search_tree;
  for (size_t j = 0; j < control_points.size(); j++) {
    search_tree.addPoint(control_points[j], values.exists(gtsam::Symbol(prefix, j)));
  }

  if (search_tree.getLeafCount() < k) {
    SPARK_LOG(WARNING) << "Not enough valid control points to deform points";
    return;
  }

  for (size_t p_idx = 0; p_idx < num_points; ++p_idx) {
    const size_t ii = indices ? indices->at(p_idx) : p_idx;
    control_point_map.emplace_back();
    const auto p_new = interpPoint(control_point_map.back(),
                                   prefix,
                                   control_points,
                                   values,
                                   search_tree,
                                   k,
                                   traits::get_vertex(points, ii));
    traits::set_vertex(new_points, ii, p_new);
  }
}

/*! \brief Deform a points (i.e. the vertices of a mesh) based on the
 * controls points via deformation but also check timestamp of points
 * - original_points: set of points to deform
 * - stamps: timestamps of the points to deform
 * - prefix: a char to distinguish the type of control points
 * - control_points: original positions of the control points. In the case of
 * mesh vertices, these are the original positions of the simplified mesh.
 * - control_point_stamps: timestamps of the control points
 * - values: key-value pairs. Where each key should be gtsam::Symbol(prefix,
 * idx-in-control-points) from the previous two arguments.
 * - k: how many nearby nodes to use to adjust new position of vertices
 * - tol_t: time (in seconds) minimum difference in time that a control point
 * can be used for interpolation
 */
template <typename CloudOut,
          typename CloudIn,
          std::enable_if_t<traits::has_get_stamp<CloudIn>::value, bool> = true>
void deformPoints(CloudOut& new_points,
                  std::vector<std::set<size_t>>& control_point_map,
                  const CloudIn& points,
                  char prefix,
                  const std::vector<gtsam::Point3>& control_points,
                  const std::vector<Timestamp>& control_point_stamps,
                  const gtsam::Values& values,
                  size_t k = 4,
                  double tol_t = 10.0,
                  const std::vector<size_t>* indices = nullptr) {
  // Check if there are points to deform
  const size_t num_points = indices ? indices->size() : traits::num_vertices(points);
  if (!num_points) {
    return;
  }

  // Cannot deform if no nodes in the deformation graph
  if (control_points.size() < k) {
    SPARK_LOG(WARNING) << "Not enough valid control points to deform points";
    return;
  }

  control_point_map.clear();
  SearchTree search_tree;

  // By doing this implicitly assuming control_point_stamps is increasing
  // TODO(yun) check this assumption
  size_t ctrl_pt_idx = 0;
  size_t lower_ctrl_pt_idx = 0;
  for (size_t point_index = 0; point_index < num_points; ++point_index) {
    const size_t ii = indices ? indices->at(point_index) : point_index;
    const auto stamp = traits::get_timestamp(points, ii);
    size_t num_ctrl_pts = search_tree.getLeafCount();
    // Add control points to octree until both
    // exceeds interpolate horizon and have enough points to deform
    while (ctrl_pt_idx < control_points.size() &&
           (control_point_stamps[ctrl_pt_idx] <= stamp + stampFromSec(tol_t) ||
            num_ctrl_pts < k + 1)) {
      const auto ctrl_valid = values.exists(gtsam::Symbol(prefix, ctrl_pt_idx));
      search_tree.addPoint(control_points[ctrl_pt_idx], ctrl_valid);
      ctrl_pt_idx++;
      if (!ctrl_valid) {
        continue;
      }

      num_ctrl_pts++;
    }

    if (search_tree.getLeafCount() < k + 1) {
      SPARK_LOG(ERROR) << "Not enough valid control points in octree to interpolate";
      if (num_ctrl_pts > 1) {
        k = num_ctrl_pts - 1;
      } else {
        continue;
      }
    }

    control_point_map.emplace_back();
    const auto p_old = traits::get_vertex(points, ii);
    const auto p_new = interpPoint(control_point_map.back(),
                                   prefix,
                                   control_points,
                                   values,
                                   search_tree,
                                   k,
                                   p_old);
    traits::set_vertex(new_points, ii, p_new);

    size_t num_leaves = search_tree.getLeafCount();
    while (lower_ctrl_pt_idx < control_points.size() && num_leaves > k + 1 &&
           control_point_stamps[lower_ctrl_pt_idx] < stamp - stampFromSec(tol_t)) {
      if (!values.exists(gtsam::Symbol(prefix, lower_ctrl_pt_idx))) {
        lower_ctrl_pt_idx++;
        continue;
      }

      search_tree.removePoint(lower_ctrl_pt_idx);
      num_leaves--;
      lower_ctrl_pt_idx++;
    }
  }
}

}  // namespace deformation
}  // namespace kimera_pgmo
