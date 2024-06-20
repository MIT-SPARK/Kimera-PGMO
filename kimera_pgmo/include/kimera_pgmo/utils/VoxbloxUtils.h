/**
 * @file   VoxbloxUtils.h
 * @brief  Some utility functions in working with voxblox msg types
 * @author Yun Chang
 */

#pragma once

#include <voxblox/core/common.h>

#include <string>

// TODO(nathan) see what this is used for and remove
namespace std {
template <>
struct less<voxblox::BlockIndex> {
  bool operator()(const voxblox::BlockIndex& a, const voxblox::BlockIndex& b) const {
    return std::lexicographical_compare(
        a.data(), a.data() + a.size(), b.data(), b.data() + b.size());
  }
};
}  // namespace std

namespace kimera_pgmo {

/*! \brief Convert a pcl point to a voxblox longindex type for voxblox cell
 * hashing
 *  - pcl_point: pcl point type
 *  - resolution: double resolution of cells
 */
template <class point_type>
voxblox::LongIndex PclPtToVoxbloxLongIndex(const point_type& pcl_point,
                                           double resolution) {
  const double threshold_inv = 1. / resolution;
  voxblox::Point vertex;
  vertex << pcl_point.x, pcl_point.y, pcl_point.z;
  const Eigen::Vector3d scaled_vector = vertex.cast<double>() * threshold_inv;
  return voxblox::LongIndex(std::round(scaled_vector.x()),
                            std::round(scaled_vector.y()),
                            std::round(scaled_vector.z()));
}

}  // namespace kimera_pgmo
