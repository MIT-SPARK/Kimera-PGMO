#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>
namespace kimera_pgmo {

void declare_config(KimeraRpgoOptimizer::Config& config) {
  using namespace config;
  name("KimeraRpgoOptimizer");
  field(config.odom_trans_threshold, "odom_trans_threshold");
  field(config.odom_rot_threshold, "odom_rot_threshold");
  field(config.pcm_trans_threshold, "pcm_trans_threshold");
  field(config.pcm_rot_threshold, "pcm_rot_threshold");
  field(config.gnc_alpha, "gnc_alpha");
  field(config.gnc_max_it, "gnc_max_iterations");
  field(config.gnc_mu_step, "gnc_mu_step");
  field(config.gnc_cost_tol, "gnc_cost_tolerance");
  field(config.gnc_weight_tol, "gnc_weight_tolerance");
  field(config.gnc_fix_prev_inliers, "gnc_fix_prev_inliers");
  field(config.lm_diagonal_damping, "lm_diagonal_damping");
}

KimeraRpgoOptimizer::KimeraRpgoOptimizer(const Config& config)
    : config(config::checkValid(config)) {
  // Initialize RPGO
  pgo_params_.setPcmSimple3DParams(config.odom_trans_threshold,
                                   config.odom_rot_threshold,
                                   config.pcm_trans_threshold,
                                   config.pcm_rot_threshold,
                                   KimeraRPGO::Verbosity::UPDATE);
  pgo_params_.setLmDiagonalDamping(config.lm_diagonal_damping);

  // Use GNC (confidence value)
  if (config.gnc_alpha > 0 && config.gnc_alpha < 1) {
    pgo_params_.setGncInlierCostThresholdsAtProbability(
        config.gnc_alpha,
        static_cast<size_t>(config.gnc_max_it),
        config.gnc_mu_step,
        config.gnc_cost_tol,
        config.gnc_weight_tol,
        config.gnc_fix_prev_inliers);
  }
  pgo_ = std::unique_ptr<KimeraRPGO::RobustSolver>(
      new KimeraRPGO::RobustSolver(pgo_params_));
}

KimeraRpgoOptimizer::~KimeraRpgoOptimizer() {}

void KimeraRpgoOptimizer::update(const Factors& factors,
                                 const Values& initial,
                                 const Factors* temp_factors,
                                 const Values* temp_initial) {
  pgo_.reset(new KimeraRPGO::RobustSolver(pgo_params_));
  if (temp_factors && temp_initial) {
    pgo_->updateTempFactorsValues(*temp_factors, *temp_initial);
  }
  pgo_->update(factors, initial);
  // NOTE(Yun): This might not give the correct behavior if using PCM
  inlier_weights_ = pgo_->getGncWeights();
  temp_inlier_weights_ = pgo_->getGncTempWeights();
  result_ = pgo_->calculateEstimate();
  temp_result_ = pgo_->getTempValues();
}

KimeraRpgoOptimizer::Values KimeraRpgoOptimizer::getEstimates() { return result_; }

KimeraRpgoOptimizer::Values KimeraRpgoOptimizer::getTempEstimates() {
  return temp_result_;
}

KimeraRpgoOptimizer::Vector KimeraRpgoOptimizer::getInlierWeights() {
  return inlier_weights_;
}

KimeraRpgoOptimizer::Vector KimeraRpgoOptimizer::getTempInlierWeights() {
  return temp_inlier_weights_;
}

void KimeraRpgoOptimizer::setLogPath(const std::string& log_path) {
  Optimizer::setLogPath(log_path);
  pgo_params_.logOutput(log_path);
  pgo_.reset(new KimeraRPGO::RobustSolver(pgo_params_));
}
}  // namespace kimera_pgmo
