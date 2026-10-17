#pragma once

#include <memory>
#include <string>

#include "kimera_pgmo/optimization_types.h"
namespace kimera_pgmo {
class Optimizer {
 public:
  using Ptr = std::unique_ptr<Optimizer>;

  virtual ~Optimizer() = default;

  // Calls on one optimizer instance must be externally serialized.
  // Extras may reference base keys, but initial value keys must be disjoint.
  // Callers supply compatible factor/value types; no representation conversion
  // occurs here. Results preserve those types and the input factor ordering.
  virtual OptimizationResult update(const OptimizationProblem& problem,
                                    const OptimizationProblem& extras = {}) = 0;

  virtual void setLogPath(const std::string& log_path) { log_path_ = log_path; }

 protected:
  std::string log_path_;
};
}  // namespace kimera_pgmo
