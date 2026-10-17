#include "kimera_pgmo/optimization_types.h"

namespace kimera_pgmo {

bool OptimizationProblem::valid(bool check_factors) const {
  for (const auto idx : inliers) {
    if (idx >= factors.size()) {
      return false;
    }
  }

  if (!check_factors) {
    return true;
  }

  for (const auto& factor : factors) {
    for (const auto& key : factor->keys()) {
      if (!initial.exists(key)) {
        return false;
      }
    }
  }

  return true;
}

}  // namespace kimera_pgmo
