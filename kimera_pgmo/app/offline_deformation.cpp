#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <config_utilities/virtual_config.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/optimizer/kimera_rpgo_optimizer.h>

namespace kimera_pgmo {

struct OfflineDeformationConfig {
  std::string dgrf_file;
  bool use_4dof_optim = false;
  std::string log_path;
  // optimizer
  config::VirtualConfig<Optimizer> optimizer{KimeraRpgoOptimizer::Config()};
};

void declare_config(OfflineDeformationConfig& config) {
  using namespace config;
  name("OfflineDeformationConfig");
  field(config.dgrf_file, "dgrf_file");
  field(config.log_path, "log_path");
  field(config.optimizer, "optimizer");
  field(config.use_4dof_optim, "use_4dof_optim");
}

class OfflineDeformation {
 public:
  OfflineDeformation(const OfflineDeformationConfig& config)
      : config_(config),
        pgo_(config.optimizer.create()),
        deformation_graph_(DeformationGraph::load(config.log_path)) {
    std::cout << "[OfflineDeformation] Initialized with:\n"
              << config::toString(config) << std::endl;
    pgo_->setLogPath(config.log_path);
    std::cout << "Loaded " << config.dgrf_file << std::endl;
  }

  void run() {
    const auto result = pgo_->update(
        deformation_graph_->getOptimizationProblem(config_.use_4dof_optim));
    deformation_graph_->updateOptimizationResult(result);
    if (!config_.log_path.empty()) {
      deformation_graph_->save(config_.log_path);
      std::cout << "Saved deformation optimization result to " << config_.log_path
                << std::endl;
    }
  }

 private:
  OfflineDeformationConfig config_;
  Optimizer::Ptr pgo_;
  DeformationGraph::Ptr deformation_graph_;
};

}  // namespace kimera_pgmo

int main(int argc, char* argv[]) {
  /*
   * Usage: ./offline_deformation path/to/dgrf [path/to/output/folder]
   * --config-utilities-file path/to/config
   */
  config::initContext(argc, argv, true);

  auto config =
      config::checkValid(config::fromContext<kimera_pgmo::OfflineDeformationConfig>());

  config.dgrf_file = argv[1];

  if (argc > 2) {
    config.log_path = argv[2];
  }

  kimera_pgmo::OfflineDeformation module(config);
  module.run();
}
