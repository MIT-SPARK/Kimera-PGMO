#pragma once
#include <config_utilities/factory.h>
#include <kimera_rpgo/rpgo.h>

#include "kimera_pgmo/optimizer/optimizer_interface.h"

namespace kimera_pgmo {

class KimeraRpgoOptimizer : public Optimizer {
 public:
  struct Config {
    kimera_rpgo::SolverConfig::LeastSquaresOption solver;
    bool use_gnc = true;
    kimera_rpgo::GncParams gnc;
    bool use_pcm = false;
    kimera_rpgo::PcmConfig pcm;
    int verbosity = 0;
  } const config;

  KimeraRpgoOptimizer(const Config& config);

  ~KimeraRpgoOptimizer() override;

  void update(const Factors& factors,
              const Values& initial,
              const Factors* temp_factors = nullptr,
              const Values* temp_initial = nullptr) override;

  const Values& getEstimates() const override;
  const Values& getTempEstimates() const override;
  const std::vector<double>& getInlierWeights() const override;
  const std::vector<double>& getTempInlierWeights() const override;

 private:
  void setLogPath(const std::string& log_path) override;

 private:
  std::unique_ptr<kimera_rpgo::Rpgo> rpgo_;
  kimera_rpgo::RpgoConfig rpgo_config_;

  Values result_;
  Values temp_result_;
  std::vector<double> inlier_weights_;
  std::vector<double> temp_inlier_weights_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<Optimizer,
                                     KimeraRpgoOptimizer,
                                     KimeraRpgoOptimizer::Config>("KimeraRpgoOptimizer");
};
void declare_config(KimeraRpgoOptimizer::Config& config);
}  // namespace kimera_pgmo
