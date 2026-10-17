#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <set>
#include <vector>

namespace kimera_pgmo {

//! Optimzer problem setup
struct OptimizationProblem {
  //! Initial values of decision variables
  gtsam::Values initial;
  //! Factor information for optimization
  gtsam::NonlinearFactorGraph factors;
  //! Indices of factors that are known optimizers
  std::set<size_t> inliers;

  //! Verify that a problem is valid
  bool valid(bool check_factors = true) const;
};

//! Estimates resulting from optimization
struct OptimizationResult {
  //! Decision variable estimates
  gtsam::Values estimates;
  //! Decision variable estimates for extra decision variables
  gtsam::Values extra_estimates;
  //! Factor weights in original problem
  std::vector<double> weights;
  //! Factor weights for extra factors
  std::vector<double> extra_weights;
};

}  // namespace kimera_pgmo
