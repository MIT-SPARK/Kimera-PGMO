#include "kimera_pgmo/optimizer/kimera_rpgo_optimizer.h"

#include <config_utilities/config.h>
#include <config_utilities/factory.h>
#include <config_utilities/types/enum.h>
#include <config_utilities/validation.h>

#include <algorithm>
#include <iomanip>

#include "kimera_pgmo/deformation_graph_4dof.h"

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

void addProblem(kimera_rpgo::Rpgo& rpgo,
                const Optimizer::Problem& problem,
                bool use_4dof) {
  if (!use_4dof) {
    rpgo.addFactors(problem.factors);
    rpgo.addValues(problem.initial);
  }

  const auto [factors, init] = convertGraphToPose4DoF(problem.factors, problem.initial);
  rpgo.addFactors(problem.factors);
  rpgo.addValues(problem.initial);
}

std::string printIterations(const std::vector<IterInfo>& iterations) {
  if (iterations.empty()) {
    return "";
  }

  std::stringstream ss;
  ss << std::string(80, '=') << "\n";
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
  field(config.use_4dof_optim, "use_4dof_optim");
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

auto KimeraRpgoOptimizer::update(const Problem& problem, const Problem& extra)
    -> Result {
  rpgo_->clear();

  std::vector<IterInfo> iterations;
  rpgo_->setIterationCallback([&](size_t iteration, double prev, double curr) {
    iterations.push_back({iteration, prev, curr});
  });

  addProblem(*rpgo_, problem, config.use_4dof_optim);
  addProblem(*rpgo_, extra, config.use_4dof_optim);

  std::set<size_t> all_known_inliers = problem.inliers;
  size_t n = problem.factors.size();
  for (const auto idx : extra.inliers) {
    all_known_inliers.insert(idx + n);
  }

  rpgo_->setKnownInliers(all_known_inliers);
  rpgo_->run();

  auto weights = rpgo_->getInlierWeights();
  auto rpgo_result = config.use_4dof_optim ? convertPose4DoFToPose3(rpgo_->getResult())
                                           : rpgo_->getResult();

  Result result;
  result.weights = std::vector<double>(weights.begin(), weights.begin() + n);
  result.extra_weights = std::vector<double>(weights.begin() + n, weights.end());
  for (const auto& [key, value] : rpgo_result) {
    if (problem.initial.exists(key)) {
      result.estimates.update(key, value);
    }

    if (extra.initial.exists(key)) {
      result.extra_estimates.update(key, value);
    }
  }

  const auto is_inlier = [](double x) { return x > 0.5; };
  const auto num_inliers = std::count_if(weights.begin(), weights.end(), is_inlier);

  const auto log = rpgo_->getLog();
  std::stringstream ss;
  ss << "Optimized " << log.num_factors << " factors and " << log.num_variables
     << " variables (" << num_inliers << " inliers, " << all_known_inliers.size()
     << " known) in " << log.elapsed.count() << " [ms]";
  if (config.print_iterations && !iterations.empty()) {
    ss << printIterations(iterations);
  }

  SPARK_LOG(INFO) << ss.str();
  if (!log_path_.empty()) {
    rpgo_->writeLog(log_path_ + "/rpgo_log.json");
  }

  return result;
}

}  // namespace kimera_pgmo
