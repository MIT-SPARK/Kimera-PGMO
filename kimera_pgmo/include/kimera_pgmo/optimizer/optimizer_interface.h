#pragma once

#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <string>
namespace kimera_pgmo {
class Optimizer {
 public:
  using Ptr = std::unique_ptr<Optimizer>;
  using Factors = gtsam::NonlinearFactorGraph;
  using Values = gtsam::Values;
  using Vector = gtsam::Vector;

  virtual ~Optimizer() = default;

  virtual void update(const Factors& factors,
                      const Values& initial,
                      const std::set<size_t>& known_inliers,
                      const Factors& temp_factors,
                      const Values& temp_initial,
                      const std::set<size_t>& temp_known_inliers) = 0;

  virtual const Values& getEstimates() const = 0;
  virtual const Values& getTempEstimates() const = 0;
  virtual const std::vector<double>& getInlierWeights() const = 0;
  virtual const std::vector<double>& getTempInlierWeights() const = 0;

  virtual void setLogPath(const std::string& log_path) { log_path_ = log_path; }

 protected:
  std::string log_path_;
};
}  // namespace kimera_pgmo
