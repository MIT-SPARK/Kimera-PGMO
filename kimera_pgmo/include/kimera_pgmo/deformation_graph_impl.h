#ifndef KIMERA_PGMO_DEFORMATION_GRAPH_IMPL_H
#define KIMERA_PGMO_DEFORMATION_GRAPH_IMPL_H

#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/pcl_mesh_traits.h"
#include "kimera_pgmo/utils/logging.h"
#include "kimera_pgmo/utils/range_generator.h"

namespace kimera_pgmo {

template <typename Cloud>
size_t DeformationGraph::findStartIndex(char prefix,
                                        int start_index_hint,
                                        const Cloud& cloud,
                                        double tol_t) const {
  const bool have_prefix_vertices =
      last_calculated_vertices_.find(prefix) != last_calculated_vertices_.end();
  if (!have_prefix_vertices) {
    return 0;
  }

  if (recalculate_vertices_) {
    SPARK_LOG(INFO) << "DeformationGraph: Recalculating mesh vertices in deformMesh";
    return 0;
  }

  if (start_index_hint >= 0) {
    return start_index_hint;
  }

  Timestamp min_stamp =
      std::max(static_cast<Timestamp>(0),
               vertices_.at(prefix).back().timestamp_ns - stampFromSec(tol_t));

  RangeGenerator gen(traits::num_vertices(cloud));
  auto bound = std::upper_bound(gen.begin(), gen.end(), min_stamp, [&](auto v, auto i) {
    return v < traits::get_timestamp(cloud, i);
  });

  return std::min(static_cast<size_t>(bound - gen.begin()),
                  last_calculated_vertices_.at(prefix).size());
}

template <typename Cloud>
void DeformationGraph::fillPreviousPoints(Cloud& vertices,
                                          char prefix,
                                          size_t start_idx) const {
  auto iter = last_calculated_vertices_.find(prefix);
  if (iter == last_calculated_vertices_.end()) {
    return;
  }

  for (size_t i = 0; i < start_idx; i++) {
    traits::set_vertex(vertices, i, traits::get_vertex(iter->second, i));
  }
}

template <typename Cloud>
void DeformationGraph::cacheNewPoints(const Cloud& vertices,
                                      char prefix,
                                      size_t start_idx) {
  auto iter = last_calculated_vertices_.find(prefix);
  if (iter == last_calculated_vertices_.end()) {
    iter = last_calculated_vertices_.emplace(prefix, pcl::PointCloud<pcl::PointXYZ>())
               .first;
  }

  const auto total_vertices = traits::num_vertices(vertices);
  iter->second.resize(total_vertices);
  for (size_t i = start_idx; i < total_vertices; i++) {
    traits::set_vertex(iter->second, i, traits::get_vertex(vertices, i));
  }
}

template <typename CloudIn, typename CloudOut>
void DeformationGraph::predeformPoints(CloudOut& new_vertices,
                                       const CloudIn& vertices,
                                       const gtsam::Values& optimized_values,
                                       const std::vector<int>& graph_indices,
                                       std::vector<size_t>& indices_to_deform,
                                       char prefix,
                                       size_t start_idx) {
  const auto num_vertices = traits::num_vertices(vertices);
  for (size_t i = start_idx; i < num_vertices; i++) {
    const int index = graph_indices.at(i);
    if (index < 0 || !optimized_values.exists(gtsam::Symbol(prefix, index))) {
      // Have to check here because sometimes interpolation happen before mesh
      // graph received
      // TODO(yun) double check this
      indices_to_deform.push_back(i);
      continue;
    }

    const Eigen::Vector3d vi = traits::get_vertex(vertices, i).template cast<double>();
    const auto transform =
        optimized_values.at<gtsam::Pose3>(gtsam::Symbol(prefix, index));
    const auto gindex = vertices_[prefix].at(index).position;
    gtsam::Point3 deformed_point =
        transform.rotation().rotate(vi - gindex) + transform.translation();
    traits::set_vertex(new_vertices, i, deformed_point.cast<float>());
  }
}

template <typename CloudIn, typename CloudOut>
void DeformationGraph::deformPoints(CloudOut& vertices,
                                    const CloudIn& old_vertices,
                                    char prefix,
                                    const gtsam::Values& optimized_values,
                                    size_t k,
                                    double tol_t,
                                    const std::vector<int>* graph_indices,
                                    int start_index_hint,
                                    std::vector<std::set<size_t>>* vertex_graph_map) {
  // Cannot deform if no nodes in the deformation graph
  const auto iter = vertices_.find(prefix);
  if (iter == vertices_.end()) {
    SPARK_LOG(DEBUG) << "Deformation graph has no vertices for prefix. No deformation";
    return;
  }

  const auto start_idx = findStartIndex(prefix, start_index_hint, old_vertices, tol_t);
  fillPreviousPoints(vertices, prefix, start_idx);

  std::vector<size_t> to_deform;
  if (start_idx != 0) {
    if (graph_indices) {
      predeformPoints(vertices,
                      old_vertices,
                      optimized_values,
                      *graph_indices,
                      to_deform,
                      prefix,
                      start_idx);
    } else {
      to_deform.resize(traits::num_vertices(vertices) - start_idx);
      std::iota(to_deform.begin(), to_deform.end(), start_idx);
    }
  }

  const std::vector<size_t>* indices_ptr = start_idx == 0 ? nullptr : &to_deform;
  std::vector<std::set<size_t>> vertex_graph_map_deformed;
  deformation::deformPoints(vertices,
                            vertex_graph_map_deformed,
                            old_vertices,
                            prefix,
                            iter->second,
                            optimized_values,
                            k,
                            tol_t,
                            indices_ptr);

  if (vertex_graph_map) {
    if (start_idx == 0) {
      *vertex_graph_map = vertex_graph_map_deformed;
    } else {
      vertex_graph_map->resize(traits::num_vertices(vertices));
      for (size_t i = 0; i < indices_ptr->size(); i++) {
        vertex_graph_map->at(indices_ptr->at(i)) = vertex_graph_map_deformed.at(i);
      }
    }
  }

  cacheNewPoints(vertices, prefix, start_idx);
  recalculate_vertices_ = false;
}

template <typename CloudIn>
void DeformationGraph::customDeformation(
    const std::function<void(const Eigen::Isometry3d&, size_t)>& callback,
    const CloudIn& points,
    char prefix,
    size_t k,
    double tol_t) const {
  // Cannot deform if no nodes in the deformation graph
  if (vertex_positions_.find(prefix) == vertex_positions_.end()) {
    SPARK_LOG(DEBUG)
        << "Deformation graph has no vertices for mesh prefix. No deformation";
    return;
  }

  std::vector<std::set<size_t>> vertex_graph_map_deformed;
  deformation::processPoints(
      [&](const size_t ii,
          std::set<size_t>& control_points_seen,
          char prefix,
          const std::vector<gtsam::Point3>& control_points,
          const gtsam::Values& values,
          const deformation::SearchTree& octree,
          size_t k) {
        const auto transform = interpDeformation(control_points_seen,
                                                 prefix,
                                                 control_points,
                                                 values,
                                                 octree,
                                                 k,
                                                 traits::get_vertex(points, ii));

        callback(transform, ii);
      },
      vertex_graph_map_deformed,
      points,
      prefix,
      vertex_positions_.at(prefix),
      vertex_stamps_.at(prefix),
      info_->values,
      k,
      tol_t,
      nullptr);
}

template <typename CloudIn, typename CloudOut>
void DeformationGraph::deformAllPoints(CloudOut& vertices,
                                       const CloudIn& old_vertices,
                                       char prefix,
                                       size_t k,
                                       double tol_t) const {
  // Cannot deform if no nodes in the deformation graph
  const auto iter = vertices_.find(prefix);
  if (iter == vertices_.end()) {
    SPARK_LOG(DEBUG) << "Deformation graph has no vertices for prefix. No deformation";
    return;
  }

  std::vector<std::set<size_t>> vertex_graph_map_deformed;
  deformation::deformPoints(vertices,
                            vertex_graph_map_deformed,
                            old_vertices,
                            prefix,
                            iter->second,
                            info_.values,
                            k,
                            tol_t,
                            nullptr);
}

}  // namespace kimera_pgmo

#endif  // KIMERA_PGMO_DEFORMATION_GRAPH_IMPL_H
