#include <config_utilities/config.h>
#include <config_utilities/parsing/yaml.h>
#include <config_utilities/printing.h>
#include <config_utilities/validation.h>
#include <config_utilities/virtual_config.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <kimera_pgmo/deformation_graph.h>
#include <kimera_pgmo/optimizer/kimera_rpgo_optimizer.h>

#include <filesystem>
DEFINE_string(dgrf, "", "dgrf file to read");
DEFINE_string(config, "", "path to config to use");
DEFINE_string(log_path, "", "dgrf file to read");

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
    pgo_->update(*deformation_graph_->getFactors(),
                 *deformation_graph_->getValues(),
                 deformation_graph_->getTempFactors(),
                 deformation_graph_->getTempValues(),
                 deformation_graph_->getKnownInlierSet());
    auto estimates = pgo_->getEstimates();
    auto temp_estimates = pgo_->getTempEstimates();
    auto inlier_weights = pgo_->getInlierWeights();
    auto temp_inlier_weights = pgo_->getTempInlierWeights();
    deformation_graph_->updateValues(estimates);
    deformation_graph_->updateTempValues(temp_estimates);
    deformation_graph_->updateInlierWeights(inlier_weights);
    deformation_graph_->updateTempInlierWeights(temp_inlier_weights);
    deformation_graph_->save(config_.log_path);
    LOG(INFO) << "Saved deformation optimization result to " << config_.log_path;
  }

 private:
  OfflineDeformationConfig config_;
  Optimizer::Ptr pgo_;
  DeformationGraphPtr deformation_graph_;
};

}  // namespace kimera_pgmo

int main(int argc, char* argv[]) {
  FLAGS_minloglevel = 0;
  FLAGS_logtostderr = 1;
  FLAGS_colorlogtostderr = 1;

  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  YAML::Node node;
  std::filesystem::path config_path = FLAGS_config;
  if (!config_path.empty() && std::filesystem::exists(config_path)) {
    node = YAML::LoadFile(config_path);
  }

  auto config = config::fromYaml<kimera_pgmo::OfflineDeformationConfig>(node);
  config.dgrf_file = FLAGS_dgrf;
  config.log_path = FLAGS_log_path;

  kimera_pgmo::OfflineDeformation module(config);
  module.run();
}
