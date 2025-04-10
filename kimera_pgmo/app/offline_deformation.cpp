#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <config_utilities/virtual_config.h>
#include <glog/logging.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/optimizer/kimera_rpgo_optimizer.h>

#include <filesystem>

namespace kimera_pgmo {

struct OfflineDeformationConfig {
  std::string dgrf_file;
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
}

class OfflineDeformation {
 public:
  OfflineDeformation(const OfflineDeformationConfig& config)
      : config_(config),
        pgo_(config.optimizer.create()),
        deformation_graph_(new DeformationGraph) {
    LOG(INFO) << "[OfflineDeformation] Initialized with:\n" << config::toString(config);
    pgo_->setLogPath(config.log_path);
    deformation_graph_->load(config.dgrf_file);
    LOG(INFO) << "Loaded " << config.dgrf_file;
  }

  void run() {
    pgo_->update(deformation_graph_->getFactorsCopy(),
                 deformation_graph_->getValuesCopy(),
                 deformation_graph_->getKnownInlierSetCopy(),
                 deformation_graph_->getTempFactorsCopy(),
                 deformation_graph_->getTempValuesCopy(),
                 deformation_graph_->getTempKnownInlierSetCopy());
    auto estimates = pgo_->getEstimates();
    auto temp_estimates = pgo_->getTempEstimates();
    auto inlier_weights = pgo_->getInlierWeights();
    auto temp_inlier_weights = pgo_->getTempInlierWeights();
    deformation_graph_->updateValues(estimates);
    deformation_graph_->updateTempValues(temp_estimates);
    deformation_graph_->updateInlierWeights(inlier_weights);
    deformation_graph_->updateTempInlierWeights(temp_inlier_weights);
    if (!config_.log_path.empty()) {
      deformation_graph_->save(config_.log_path);
      LOG(INFO) << "Saved deformation optimization result to " << config_.log_path;
    }
  }

 private:
  OfflineDeformationConfig config_;
  Optimizer::Ptr pgo_;
  DeformationGraphPtr deformation_graph_;
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
