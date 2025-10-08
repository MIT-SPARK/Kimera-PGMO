#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"

#include <config_utilities/config.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>
#include <glog/logging.h>

#include <algorithm>
#include <iomanip>

#include "kimera_pgmo/deformation_graph_4dof.h"

namespace kimera_pgmo {
namespace {

static const auto registration =
    config::RegistrationWithConfig<Optimizer,
                                   KimeraRpgoOptimizer,
                                   KimeraRpgoOptimizer::Config>("KimeraRpgoOptimizer");
}

using namespace kimera_rpgo;

void declare_config(KimeraRpgoOptimizer::Config& config) {
  using namespace config;
  name("KimeraRpgoOptimizer");
  enum_field(config.solver,
             "solver",
             {{SolverConfig::LeastSquaresOption::LM, "LM"},
              {SolverConfig::LeastSquaresOption::GN, "GN"},
              {SolverConfig::LeastSquaresOption::DOGLEG, "DOGLEG"}});
  field(config.verbosity, "verbosity");
  field(config.print_summary, "print_summary");
  field(config.print_iterations, "print_iterations");
  field(config.use_4dof_optim, "use_4dof_optim");
  field(config.use_gnc, "use_gnc");
  {
    NameSpace ns("gnc");
    field(config.gnc.barc_sq, "barc_sq");
    field(config.gnc.inlier_probability, "inlier_probability");
    field(config.gnc.mu_step, "mu_step");
    field(config.gnc.max_iterations, "max_iterations");
    field(config.gnc.fix_odom, "fix_odom");
    enum_field(config.gnc.robust_cost,
               "robust_cost",
               {{LossType::TLS, "TLS"}, {LossType::GM, "GM"}});
  }
  field(config.use_pcm, "use_pcm");
  {
    NameSpace ns("pcm");
    field(config.pcm.trans_threshold, "trans_threshold");
    field(config.pcm.rot_threshold, "rot_threshold");
    field(config.pcm.mahalanobis_threshold, "mahalanobis_threshold");
    field(config.pcm.odom_check, "odom_check");
    field(config.pcm.odom_trans_threshold, "odom_trans_threshold");
    field(config.pcm.odom_rot_threshold, "odom_rot_threshold");
    field(config.pcm.odom_mahalanobis_threshold, "odom_mahalanobis_threshold");
    enum_field(config.pcm.max_clique_mode,
               "max_clique_mode",
               {{PcmConfig::MaxCliqueMode::EXACT, "EXACT"},
                {PcmConfig::MaxCliqueMode::HEURISTIC, "HEURISTIC"},
                {PcmConfig::MaxCliqueMode::INCREMENTAL, "INCREMENTAL"}});
    enum_field(config.pcm.metric_type,
               "metric_type",
               {{PcmConfig::MetricType::NODE, "NODE"},
                {PcmConfig::MetricType::COVARIANCE, "COVARIANCE"}});
  }
}

KimeraRpgoOptimizer::KimeraRpgoOptimizer(const Config& config)
    : config(config::checkValid(config)) {
  // Initialize RPGO
  rpgo_config_.print_summary = config.print_summary;
  rpgo_config_.solver_config.least_squares_option = config.solver;
  rpgo_config_.solver_config.verbosity = config.verbosity;
  rpgo_config_.solver_config.setLeastSquaresParamsDefault();

  if (config.use_gnc) {
    rpgo_config_.solver_config.setGncParams(config.gnc);
  }

  if (config.use_pcm) {
    rpgo_config_.use_pcm = true;
    rpgo_config_.pcm_config = config.pcm;
  }

  rpgo_.reset(new Rpgo(rpgo_config_, false));
}

KimeraRpgoOptimizer::~KimeraRpgoOptimizer() {}

struct IterInfo {
  size_t iteration;
  double prev_error;
  double curr_error;
};

void KimeraRpgoOptimizer::update(const Factors& factors,
                                 const Values& initial,
                                 const std::set<size_t>& known_inliers,
                                 const Factors& temp_factors,
                                 const Values& temp_initial,
                                 const std::set<size_t>& temp_known_inliers) {
  rpgo_->clear();

  std::vector<IterInfo> iterations;
  rpgo_->setIterationCallback([&](size_t iteration, double prev, double curr) {
    iterations.push_back({iteration, prev, curr});
  });

  const auto& [input_factors, input_initial] =
      processFactorsAndValues(factors, initial, config.use_4dof_optim);
  rpgo_->addFactors(input_factors);
  rpgo_->addValues(input_initial);

  // NOTE(hlim) The final output should be gtsam::Pose3 for compatibility,
  // regardless of whether 4DoF optimization is used or not.
  result_ = initial;

  const auto& [input_temp_factors, input_temp_initial] =
      processFactorsAndValues(temp_factors, temp_initial, config.use_4dof_optim);

  rpgo_->addFactors(input_temp_factors);
  rpgo_->addValues(input_temp_initial);
  temp_result_ = temp_initial;

  std::set<size_t> all_known_inliers = known_inliers;
  size_t n = factors.size();
  for (const auto& temp_idx : temp_known_inliers) {
    all_known_inliers.insert(temp_idx + n);
  }
  rpgo_->setKnownInliers(all_known_inliers);

  // Run optimizer
  rpgo_->run();

  // Get estimates
  auto rpgo_result = config.use_4dof_optim ? convertPose4DoFToPose3(rpgo_->getResult())
                                           : rpgo_->getResult();
  for (const auto& key_val : rpgo_result) {
    if (result_.exists(key_val.key)) {
      result_.update(key_val.key, key_val.value);
    }
    if (temp_result_.exists(key_val.key)) {
      temp_result_.update(key_val.key, key_val.value);
    }
  }

  // Get inlier weights
  auto inlier_weights = rpgo_->getInlierWeights();
  inlier_weights_ = std::vector<double>(inlier_weights.begin(),
                                        inlier_weights.begin() + factors.size());
  temp_inlier_weights_ = std::vector<double>(inlier_weights.end() - temp_factors.size(),
                                             inlier_weights.end());

  int num_inliers = std::count_if(inlier_weights.begin(),
                                  inlier_weights.end(),
                                  [](double val) { return val > 0.5; });

  const auto last_log = rpgo_->getLog();
  std::stringstream ss;
  ss << "Optimized " << last_log.num_factors << " factors and "
     << last_log.num_variables << " variables (" << num_inliers << " inliers, "
     << all_known_inliers.size() << " known) in " << last_log.elapsed.count()
     << " [ms]";
  if (config.print_iterations) {
    ss << "\n" << std::string(80, '=') << "\n";
    for (const auto iter : iterations) {
      if (iter.iteration == 1) {
        ss << std::string(80, '-') << "\n";
        ss << "Starting cost: " << std::setprecision(3) << iter.prev_error << "\n";
        ss << std::string(80, '-') << "\n";
      }

      ss << "Iteration " << iter.iteration << ": " << std::setprecision(3)
         << std::scientific << iter.curr_error
         << " (change: " << iter.prev_error - iter.curr_error << ")\n";
    }

    ss << std::string(80, '=') << "\n";
  }

  LOG(INFO) << ss.str();

  if (!log_path_.empty()) {
    rpgo_->writeLog(log_path_ + "/rpgo_log.json");
  }
}

const std::pair<KimeraRpgoOptimizer::Factors, KimeraRpgoOptimizer::Values>
KimeraRpgoOptimizer::processFactorsAndValues(
    const KimeraRpgoOptimizer::Factors& factors,
    const KimeraRpgoOptimizer::Values& initial,
    const bool use_4dof_optim) {
  if (!use_4dof_optim) {
    return {factors, initial};
  }
  return convertGraphToPose4DoF(factors, initial);
}

const KimeraRpgoOptimizer::Values& KimeraRpgoOptimizer::getEstimates() const {
  return result_;
}

const KimeraRpgoOptimizer::Values& KimeraRpgoOptimizer::getTempEstimates() const {
  return temp_result_;
}

const std::vector<double>& KimeraRpgoOptimizer::getInlierWeights() const {
  return inlier_weights_;
}

const std::vector<double>& KimeraRpgoOptimizer::getTempInlierWeights() const {
  return temp_inlier_weights_;
}

void KimeraRpgoOptimizer::setLogPath(const std::string& log_path) {
  Optimizer::setLogPath(log_path);
}

}  // namespace kimera_pgmo
