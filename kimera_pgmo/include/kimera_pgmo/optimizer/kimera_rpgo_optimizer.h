#pragma once
#include <kimera_rpgo/rpgo.h>

#include "kimera_pgmo/optimizer/optimizer_interface.h"

namespace kimera_pgmo {

class KimeraRpgoOptimizer : public Optimizer {
 public:
  struct Config {
    using SolverType = kimera_rpgo::SolverConfig::LeastSquaresOption;
    SolverType solver = SolverType::LM;
    bool use_4dof_optim = false;
    bool use_gnc = true;
    kimera_rpgo::GncParams gnc;
    bool use_pcm = false;
    kimera_rpgo::PcmConfig pcm;
    bool print_summary = true;
    bool print_iterations = true;
    int verbosity = 0;
  } const config;

  explicit KimeraRpgoOptimizer(const Config& config);
  virtual ~KimeraRpgoOptimizer() = default;

  Result update(const Problem& problem, const Problem& extra) override;

 private:
  kimera_rpgo::RpgoConfig rpgo_config_;
  std::unique_ptr<kimera_rpgo::Rpgo> rpgo_;
};

void declare_config(KimeraRpgoOptimizer::Config& config);

}  // namespace kimera_pgmo
