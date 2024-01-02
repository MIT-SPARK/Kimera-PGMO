/**
 * @file   MeshDeformation.h
 * @brief  Functions for deforming mesh vertices
 * @author Yun Chang
 */

#pragma once

#include <gtsam/geometry/Point3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/console.h>

#include "kimera_pgmo/MeshTraits.h"
#include "kimera_pgmo/utils/CommonStructs.h"

namespace kimera_pgmo {
namespace deformation {

using XYZCloud = pcl::PointCloud<pcl::PointXYZ>;
using XYZOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;

template <typename Scalar>
pcl::PointXYZ eigenToPcl(const Eigen::Matrix<Scalar, 3, 1>& point) {
  return {static_cast<float>(point.x()),
          static_cast<float>(point.y()),
          static_cast<float>(point.z())};
}

// Calculate new point location from k points
traits::Pos interpPoint(std::set<size_t>& control_points_seen,
                        char prefix,
                        const std::vector<gtsam::Point3>& control_points,
                        const gtsam::Values& values,
                        XYZOctree& octree,
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
    ROS_WARN("No control points. No deformation.");
    return;
  }

  control_point_map.clear();

  // Build Octree
  XYZOctree search_octree(1.0);
  XYZCloud::Ptr search_cloud(new XYZCloud());
  search_octree.setInputCloud(search_cloud);
  for (size_t j = 0; j < control_points.size(); j++) {
    search_cloud->push_back(eigenToPcl(control_points[j]));
    if (!values.exists(gtsam::Symbol(prefix, j))) {
      continue;
    }

    search_octree.addPointFromCloud(search_cloud->points.size() - 1, nullptr);
  }

  if (search_octree.getLeafCount() < k) {
    ROS_WARN("Not enough valid control points to deform points.");
    return;
  }

  for (size_t p_idx = 0; p_idx < num_points; ++p_idx) {
    const size_t ii = indices ? indices->at(p_idx) : p_idx;
    control_point_map.emplace_back();
    const auto p_new = interpPoint(control_point_map.back(),
                                   prefix,
                                   control_points,
                                   values,
                                   search_octree,
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
    ROS_WARN("Not enough valid control points to deform points.");
    return;
  }

  control_point_map.clear();

  // Build Octree
  XYZOctree search_octree(1.0);
  XYZCloud::Ptr search_cloud(new XYZCloud());
  search_octree.setInputCloud(search_cloud);

  // By doing this implicitly assuming control_point_stamps is increasing
  // TODO(yun) check this assumption
  size_t ctrl_pt_idx = 0;
  size_t lower_ctrl_pt_idx = 0;
  for (size_t point_index = 0; point_index < num_points; ++point_index) {
    const size_t ii = indices ? indices->at(point_index) : point_index;
    const auto stamp = traits::get_timestamp(points, ii);
    size_t num_ctrl_pts = search_octree.getLeafCount();
    // Add control points to octree until both
    // exceeds interpolate horizon and have enough points to deform
    while (ctrl_pt_idx < control_points.size() &&
           (control_point_stamps[ctrl_pt_idx] <= stamp + stampFromSec(tol_t) ||
            num_ctrl_pts < k + 1)) {
      search_cloud->push_back(eigenToPcl(control_points[ctrl_pt_idx]));
      if (!values.exists(gtsam::Symbol(prefix, ctrl_pt_idx))) {
        ctrl_pt_idx++;
        continue;
      }
      search_octree.addPointFromCloud(search_cloud->points.size() - 1, nullptr);
      num_ctrl_pts++;
      ctrl_pt_idx++;
    }

    if (search_octree.getLeafCount() < k + 1) {
      ROS_ERROR("Not enough valid control points in octree to interpolate point.");
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
                                   search_octree,
                                   k,
                                   p_old);
    traits::set_vertex(new_points, ii, p_new);

    size_t num_leaves = search_octree.getLeafCount();
    while (lower_ctrl_pt_idx < control_points.size() && num_leaves > k + 1 &&
           control_point_stamps[lower_ctrl_pt_idx] < stamp - stampFromSec(tol_t)) {
      if (!values.exists(gtsam::Symbol(prefix, lower_ctrl_pt_idx))) {
        lower_ctrl_pt_idx++;
        continue;
      }
      search_octree.deleteVoxelAtPoint(lower_ctrl_pt_idx);
      num_leaves--;
      lower_ctrl_pt_idx++;
    }
  }
}

}  // namespace deformation
}  // namespace kimera_pgmo
