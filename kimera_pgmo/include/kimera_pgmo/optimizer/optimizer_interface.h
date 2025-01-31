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
                      const Factors* temp_factors = nullptr,
                      const Values* temp_initial = nullptr) = 0;

  virtual Values getEstimates() = 0;
  virtual Values getTempEstimates() = 0;
  virtual Vector getInlierWeights() = 0;
  virtual Vector getTempInlierWeights() = 0;

  virtual void setLogPath(const std::string& log_path) { log_path_ = log_path; }

 protected:
  std::string log_path_;
};
}  // namespace kimera_pgmo
