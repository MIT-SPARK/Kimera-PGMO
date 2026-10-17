#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <set>
#include <vector>

namespace kimera_pgmo {

// Owning input containers. GTSAM factor objects are shared and must stay immutable.
struct OptimizationProblem {
  gtsam::Values initial;
  gtsam::NonlinearFactorGraph factors;
  //! Known inlier indices are local to this problem's factor sequence.
  std::set<size_t> inliers;
};

// Estimates retain the input value types, including Pose4DoF when supplied.
struct OptimizationResult {
  gtsam::Values estimates;
  gtsam::Values extra_estimates;
  //! Weights follow the base input factor order.
  std::vector<double> weights;
  //! Weights follow the extras input factor order.
  std::vector<double> extra_weights;
};

}  // namespace kimera_pgmo
