#pragma once
#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/mesh_deformation.h"
#include "kimera_pgmo/utils/logging.h"

namespace kimera_pgmo {

template <typename CloudIn, typename CloudOut>
void DeformationGraph::deformPoints(CloudOut& points,
                                    const CloudIn& old_points,
                                    char prefix,
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
    to_deform.resize(traits::num_vertices(points) - start_idx);
    std::iota(to_deform.begin(), to_deform.end(), start_idx);
  }

  using deformation::deformPoints;
  const std::vector<size_t>* indices = start_idx == 0 ? nullptr : &to_deform;
  deformPoints(points, old_points, prefix, iter->second, values_, k, tol_t, indices);
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
          const auto& control_points,
          const auto& values,
          const auto& octree,
          size_t k) {
        const auto p = traits::get_vertex(points, ii);
        const auto tf = interpDeformation(prefix, control_points, values, octree, k, p);
        callback(tf, ii);
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
