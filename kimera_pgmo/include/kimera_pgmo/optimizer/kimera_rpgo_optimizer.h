#pragma once
#include <kimera_rpgo/rpgo.h>

#include "kimera_pgmo/optimizer/optimizer_interface.h"

namespace kimera_pgmo {

class KimeraRpgoOptimizer : public Optimizer {
 public:
  struct Config {
    using LSQOption = kimera_rpgo::SolverConfig::LeastSquaresOption;
    LSQOption solver = LSQOption::LM;
    bool use_gnc = true;
    kimera_rpgo::GncParams gnc;
    bool use_pcm = false;
    kimera_rpgo::PcmConfig pcm;
    bool print_summary = true;
    bool print_iterations = true;
    int verbosity = 0;
  } const config;

  KimeraRpgoOptimizer(const Config& config);

  ~KimeraRpgoOptimizer() override;

  OptimizationResult update(const OptimizationProblem& problem,
                            const OptimizationProblem& extras = {}) override;

 private:
  std::unique_ptr<kimera_rpgo::Rpgo> rpgo_;
  kimera_rpgo::RpgoConfig rpgo_config_;
};

void declare_config(KimeraRpgoOptimizer::Config& config);

}  // namespace kimera_pgmo
