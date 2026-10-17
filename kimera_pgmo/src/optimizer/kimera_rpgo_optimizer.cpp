#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>

#include <algorithm>
#include <iomanip>
#include <stdexcept>

#include "kimera_pgmo/utils/logging.h"

namespace kimera_rpgo {

void declare_config(GncParams& config) {
  using namespace config;
  name("GncParams");
  field(config.barc_sq, "barc_sq");
  field(config.inlier_probability, "inlier_probability");
  field(config.mu_step, "mu_step");
  field(config.max_iterations, "max_iterations");
  field(config.fix_odom, "fix_odom");
  enum_field(config.robust_cost,
             "robust_cost",
             {{LossType::TLS, "TLS"}, {LossType::GM, "GM"}});
}

void declare_config(PcmConfig& config) {
  using namespace config;
  name("PcmConfig");
  field(config.trans_threshold, "trans_threshold");
  field(config.rot_threshold, "rot_threshold");
  field(config.mahalanobis_threshold, "mahalanobis_threshold");
  field(config.odom_check, "odom_check");
  field(config.odom_trans_threshold, "odom_trans_threshold");
  field(config.odom_rot_threshold, "odom_rot_threshold");
  field(config.odom_mahalanobis_threshold, "odom_mahalanobis_threshold");
  enum_field(config.max_clique_mode,
             "max_clique_mode",
             {{PcmConfig::MaxCliqueMode::EXACT, "EXACT"},
              {PcmConfig::MaxCliqueMode::HEURISTIC, "HEURISTIC"},
              {PcmConfig::MaxCliqueMode::INCREMENTAL, "INCREMENTAL"}});
  enum_field(config.metric_type,
             "metric_type",
             {{PcmConfig::MetricType::NODE, "NODE"},
              {PcmConfig::MetricType::COVARIANCE, "COVARIANCE"}});
}

}  // namespace kimera_rpgo

namespace kimera_pgmo {
namespace {

static const auto registration =
    config::RegistrationWithConfig<Optimizer,
                                   KimeraRpgoOptimizer,
                                   KimeraRpgoOptimizer::Config>("KimeraRpgoOptimizer");

struct IterInfo {
  size_t iteration;
  double prev_error;
  double curr_error;
};

std::string showLog(const kimera_rpgo::Rpgo& rpgo,
                    const std::vector<IterInfo>& iterations,
                    const std::set<size_t>& known_inliers,
                    const std::vector<double>& inlier_weights,
                    bool print_iterations) {
  const auto last_log = rpgo.getLog();

  const auto num_inliers = std::count_if(inlier_weights.begin(),
                                         inlier_weights.end(),
                                         [](double val) { return val > 0.5; });

  std::stringstream ss;
  ss << "Optimized " << last_log.num_factors << " factors and "
     << last_log.num_variables << " variables (" << num_inliers << " inliers, "
     << known_inliers.size() << " known) in " << last_log.elapsed.count() << " [ms]";

  if (print_iterations) {
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

  return ss.str();
}

}  // namespace

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
  field(config.use_gnc, "use_gnc");
  field(config.gnc, "gnc");
  field(config.use_pcm, "use_pcm");
  field(config.pcm, "pcm");
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

OptimizationResult KimeraRpgoOptimizer::update(const OptimizationProblem& problem,
                                               const OptimizationProblem& extras) {
  for (const auto& value : extras.initial) {
    if (problem.initial.exists(value.key)) {
      throw std::invalid_argument("Duplicate value found");
    }
  }

  if (!problem.valid(true)) {
    throw std::invalid_argument("Problem not valid");
  }

  if (!extras.valid(false)) {
    throw std::invalid_argument("Extra factors not valid");
  }

  std::set<size_t> all_inliers = problem.inliers;
  const auto N_f = problem.factors.size();
  const auto N_e = extras.factors.size();
  for (const auto& extra_idx : extras.inliers) {
    all_inliers.insert(extra_idx + N_f);
  }

  std::vector<IterInfo> iterations;
  rpgo_->clear();
  rpgo_->setIterationCallback([&iterations](size_t iter, double prev, double curr) {
    iterations.push_back({iter, prev, curr});
  });
  rpgo_->addFactors(problem.factors);
  rpgo_->addFactors(extras.factors);
  rpgo_->addValues(problem.initial);
  rpgo_->addValues(extras.initial);
  rpgo_->setKnownInliers(all_inliers);
  rpgo_->run();

  OptimizationResult result;
  const auto& rpgo_result = rpgo_->getResult();
  for (const auto& [key, value] : rpgo_result) {
    if (problem.initial.exists(key)) {
      result.estimates.insert(key, value);
    }

    if (extras.initial.exists(key)) {
      result.extra_estimates.insert(key, value);
    }
  }

  const auto& weights = rpgo_->getInlierWeights();
  if (weights.size() != N_f + N_e) {
    throw std::runtime_error("Optimizer returned an unexpected number of weights");
  }

  result.weights = std::vector<double>(weights.begin(), weights.begin() + N_f);
  result.extra_weights = std::vector<double>(weights.begin() + N_f, weights.end());

  const auto print_iters = config.print_iterations;
  SPARK_LOG(INFO) << showLog(*rpgo_, iterations, all_inliers, weights, print_iters);
  if (!log_path_.empty()) {
    rpgo_->writeLog(log_path_ / "rpgo_log.json");
  }

  return result;
}

}  // namespace kimera_pgmo
