/**
 * @file   mesh_deformation.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#include "kimera_pgmo/mesh_deformation.h"

#include <gtsam/geometry/Pose3.h>
#include <pcl/octree/octree_search.h>

namespace kimera_pgmo {
namespace deformation {

template <typename Scalar>
pcl::PointXYZ eigenToPcl(const Eigen::Matrix<Scalar, 3, 1>& point) {
  return {static_cast<float>(point.x()),
          static_cast<float>(point.y()),
          static_cast<float>(point.z())};
}

struct SearchTree::Impl {
  using XYZCloud = pcl::PointCloud<pcl::PointXYZ>;
  using XYZOctree = pcl::octree::OctreePointCloudSearch<pcl::PointXYZ>;

  explicit Impl(double resolution)
      : search_tree(resolution), search_cloud(new XYZCloud()) {
    search_tree.setInputCloud(search_cloud);
  }

  size_t getLeafCount() const { return search_tree.getLeafCount(); }

  void addPoint(const gtsam::Point3& point, bool valid) {
    search_cloud->push_back(eigenToPcl(point));
    if (valid) {
      search_tree.addPointFromCloud(search_cloud->size() - 1, nullptr);
    }
  }

  void removePoint(size_t index) { search_tree.deleteVoxelAtPoint(index); }

  void search(const traits::Pos& point,
              size_t k,
              std::vector<int>& nn_index,
              std::vector<float>& nn_sq_dist) {
    search_tree.nearestKSearch(eigenToPcl<float>(point), k + 1, nn_index, nn_sq_dist);
  }

  XYZOctree search_tree;
  XYZCloud::Ptr search_cloud;
};

SearchTree::SearchTree(double resolution) : impl_(std::make_unique<Impl>(resolution)) {}

SearchTree::~SearchTree() {}

size_t SearchTree::getLeafCount() const { return impl_->getLeafCount(); }

void SearchTree::addPoint(const gtsam::Point3& point, bool valid) {
  return impl_->addPoint(point, valid);
}

void SearchTree::removePoint(size_t index) { impl_->removePoint(index); }

void SearchTree::search(const traits::Pos& point,
                        size_t k,
                        std::vector<int>& nn_index,
                        std::vector<float>& nn_sq_dist) const {
  impl_->search(point, k, nn_index, nn_sq_dist);
}

// Calculate new point location from k points
Eigen::Isometry3d interpDeformation(std::set<size_t>& control_points_seen,
                                    char prefix,
                                    const std::vector<gtsam::Point3>& control_points,
                                    const gtsam::Values& values,
                                    const SearchTree& tree,
                                    size_t k,
                                    const traits::Pos& old_point) {
  // Query octree
  std::vector<int> nn_index;
  std::vector<float> nn_sq_dist;
  tree.search(old_point, k + 1, nn_index, nn_sq_dist);

  const double d_max = std::sqrt(nn_sq_dist[nn_index.size() - 1]);
  bool use_const_weight = std::sqrt(nn_sq_dist[0]) == d_max || d_max == 0;

  // NOTE(nathan) weighted average of k rotations is not necessarily a rotation,
  // but if the rotations are close enough together should be pretty close. Potentially
  // worth trying slerp on all the rotations if large rotation differences are expected
  double weight_sum = 0;
  Eigen::Vector4d q = Eigen::Vector4d::Zero();
  Eigen::Vector3d t = Eigen::Vector3d::Zero();
  for (size_t j = 0; j < nn_index.size() - 1; j++) {
    double w = use_const_weight ? 1 : (1 - std::sqrt(nn_sq_dist[j]) / d_max);
    weight_sum += w;

    const auto local_tf = values.at<gtsam::Pose3>(gtsam::Symbol(prefix, nn_index[j]));
    const auto local_rot = local_tf.rotation();
    q += w * local_rot.toQuaternion().coeffs();
    t += w * (local_tf.translation() - local_rot * control_points.at(nn_index[j]));
    control_points_seen.insert(nn_index[j]);
  }

  q /= weight_sum;
  q.normalize();  // project to the nearest rotation
  t /= weight_sum;
  return Eigen::Translation3d(t) * Eigen::Quaterniond(q);
}

// Calculate new point location from k points
traits::Pos interpPoint(std::set<size_t>& control_points_seen,
                        char prefix,
                        const std::vector<gtsam::Point3>& control_points,
                        const gtsam::Values& values,
                        const SearchTree& tree,
                        size_t k,
                        const traits::Pos& vi) {
  const auto new_T_old = interpDeformation(
      control_points_seen, prefix, control_points, values, tree, k, vi);
  return new_T_old.cast<float>() * vi;
}

}  // namespace deformation
}  // namespace kimera_pgmo
