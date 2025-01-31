#pragma once
#include <KimeraRPGO/RobustSolver.h>
#include <config_utilities/factory.h>

#include "kimera_pgmo/optimizer/optimizer_interface.h"

namespace kimera_pgmo {

class KimeraRpgoOptimizer : public Optimizer {
 public:
  struct Config {
    // pcm thresholds
    double odom_trans_threshold;
    double odom_rot_threshold;
    double pcm_trans_threshold;
    double pcm_rot_threshold;
    // gnc configuration
    double gnc_alpha;
    int gnc_max_it = 100;
    double gnc_mu_step = 1.4;
    double gnc_cost_tol = 1.0e-5;
    double gnc_weight_tol = 1.0e-4;
    bool lm_diagonal_damping = true;
    bool gnc_fix_prev_inliers = false;
  } const config;

  KimeraRpgoOptimizer(const Config& config);

  ~KimeraRpgoOptimizer() override;

  void update(const Factors& factors,
              const Values& initial,
              const Factors* temp_factors = nullptr,
              const Values* temp_initial = nullptr) override;

  Values getEstimates() override;
  Values getTempEstimates() override;
  Vector getInlierWeights() override;
  Vector getTempInlierWeights() override;

 private:
  void setLogPath(const std::string& log_path) override;

 private:
  std::unique_ptr<KimeraRPGO::RobustSolver> pgo_;
  KimeraRPGO::RobustSolverParams pgo_params_;

  Values result_;
  Values temp_result_;
  Vector inlier_weights_;
  Vector temp_inlier_weights_;

 private:
  inline static const auto registration_ =
      config::RegistrationWithConfig<Optimizer,
                                     KimeraRpgoOptimizer,
                                     KimeraRpgoOptimizer::Config>(
          "KimeraRpgoOptimizer");
};

void declare_config(KimeraRpgoOptimizer::Config& config);
}  // namespace kimera_pgmo
