#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <string>

namespace kimera_pgmo {

class Optimizer {
 public:
  using Ptr = std::unique_ptr<Optimizer>;

  struct Problem {
    gtsam::Values initial;
    gtsam::NonlinearFactorGraph factors;
    std::set<size_t> inliers;
  };

  struct Result {
    gtsam::Values estimates;
    gtsam::Values extra_estimates;
    std::vector<double> weights;
    std::vector<double> extra_weights;
  };

  virtual ~Optimizer() = default;

  virtual Result update(const Problem& problem) { return update(problem, {}); }
  virtual Result update(const Problem& problem, const Problem& extra) = 0;

  virtual void setLogPath(const std::string& log_path) { log_path_ = log_path; }

 protected:
  std::string log_path_;
};

}  // namespace kimera_pgmo
