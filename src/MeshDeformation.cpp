/**
 * @file   MeshDeformation.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#include "kimera_pgmo/MeshDeformation.h"

#include <gtsam/geometry/Pose3.h>

namespace kimera_pgmo {
namespace deformation {

using XYZCloud = pcl::PointCloud<pcl::PointXYZ>;
using Octree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;

// Calculate new point location from k points
traits::Pos interpPoint(std::set<size_t>& control_points_seen,
                        char prefix,
                        const std::vector<gtsam::Point3>& control_points,
                        const gtsam::Values& values,
                        Octree& octree,
                        size_t k,
                        const traits::Pos& old_point) {
  // Query octree
  std::vector<int> nn_index;
  std::vector<float> nn_sq_dist;
  octree.nearestKSearch(eigenToPcl<float>(old_point), k + 1, nn_index, nn_sq_dist);

  const double d_max = std::sqrt(nn_sq_dist[nn_index.size() - 1]);
  bool use_const_weight = std::sqrt(nn_sq_dist[0]) == d_max || d_max == 0;

  double weight_sum = 0;
  gtsam::Point3 new_point = gtsam::Point3::Zero();
  const gtsam::Point3 vi = old_point.cast<double>();
  for (size_t j = 0; j < nn_index.size() - 1; j++) {
    const auto& gj = control_points.at(nn_index[j]);

    double w = use_const_weight ? 1 : (1 - std::sqrt(nn_sq_dist[j]) / d_max);
    weight_sum += w;
    auto transform = values.at<gtsam::Pose3>(gtsam::Symbol(prefix, nn_index[j]));
    const auto delta =
        w * (transform.rotation().rotate(vi - gj) + transform.translation());

    new_point += delta;
    control_points_seen.insert(nn_index[j]);
  }

  new_point /= weight_sum;
  return new_point.cast<float>();
}

}  // namespace deformation
}  // namespace kimera_pgmo
