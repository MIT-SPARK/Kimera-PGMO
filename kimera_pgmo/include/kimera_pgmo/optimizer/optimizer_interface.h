#pragma once

#include <filesystem>
#include <memory>

#include "kimera_pgmo/optimization_types.h"

namespace kimera_pgmo {

class Optimizer {
 public:
  using Ptr = std::unique_ptr<Optimizer>;

  virtual ~Optimizer() = default;

  virtual OptimizationResult update(const OptimizationProblem& problem,
                                    const OptimizationProblem& extras = {}) = 0;

  virtual void setLogPath(const std::filesystem::path& log_path) {
    log_path_ = log_path;
  }

 protected:
  std::filesystem::path log_path_;
};

}  // namespace kimera_pgmo
