#pragma once
#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/mesh_deformation.h"
#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {

template <typename CloudIn, typename CloudOut>
void DeformationGraph::deformPoints(CloudOut& vertices,
                                    const CloudIn& old_vertices,
                                    char prefix,
                                    size_t k,
                                    double tol_t,
                                    int start_index_hint) const {
  deformPoints(vertices, old_vertices, prefix, values_, k, tol_t, start_index_hint);
}

template <typename CloudIn, typename CloudOut>
void DeformationGraph::deformPoints(CloudOut& vertices,
                                    const CloudIn& old_vertices,
                                    char prefix,
                                    const gtsam::Values& optimized_values,
                                    size_t k,
                                    double tol_t,
                                    int start_idx) const {
  auto iter = control_points_.find(prefix);
  if (iter == control_points_.end()) {
    SPARK_LOG(DEBUG) << "Graph has no vertices for prefix. No deformation";
    return;
  }

  std::vector<size_t> to_deform;
  if (start_idx != 0) {
    to_deform.resize(traits::num_vertices(vertices) - start_idx);
    std::iota(to_deform.begin(), to_deform.end(), start_idx);
  }

  const std::vector<size_t>* indices_ptr = start_idx == 0 ? nullptr : &to_deform;
  deformation::deformPoints(vertices,
                            old_vertices,
                            prefix,
                            iter->second,
                            optimized_values,
                            k,
                            tol_t,
                            indices_ptr);
}

template <typename CloudIn>
void DeformationGraph::customDeformation(const DeformationCallback& callback,
                                         const CloudIn& points,
                                         char prefix,
                                         size_t k,
                                         double tol_t) const {
  auto iter = control_points_.find(prefix);
  if (iter == control_points_.end()) {
    SPARK_LOG(DEBUG) << "Graph has no vertices for prefix. No deformation";
    return;
  }

  deformation::processPoints(
      [&](const size_t ii,
          char prefix,
          const std::vector<gtsam::Point3>& control_points,
          const gtsam::Values& values,
          const deformation::SearchTree& octree,
          size_t k) {
        const auto transform = interpDeformation(
            prefix, control_points, values, octree, k, traits::get_vertex(points, ii));

        callback(transform, ii);
      },
      points,
      prefix,
      iter->second,
      values_,
      k,
      tol_t,
      nullptr);
}

}  // namespace kimera_pgmo
