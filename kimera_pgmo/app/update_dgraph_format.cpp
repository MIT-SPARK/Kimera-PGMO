/**
 * @file   deformation_graph_io.cpp
 * @brief  Deformation graph save and load functions
 * @author Yun Chang
 */
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <fstream>
#include <stdexcept>

#include <CLI/CLI.hpp>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph.h"

namespace kimera_pgmo {

DeformationGraph::Ptr loadDeformationGraph(const std::string& filename) {
  std::map<char, std::vector<gtsam::Pose3>> initial_poses;
  std::unordered_map<gtsam::Key, gtsam::Pose3> temp_initial_poses;
  std::map<char, std::vector<gtsam::Point3>> vertex_positions;
  std::map<char, std::vector<Timestamp>> vertex_stamps;

  auto nfg = std::make_shared<gtsam::NonlinearFactorGraph>();
  auto known_inliers = std::make_shared<std::set<size_t>>();
  auto values = std::make_shared<gtsam::Values>();

  auto temp_nfg = std::make_shared<gtsam::NonlinearFactorGraph>();
  auto temp_known_inliers = std::make_shared<std::set<size_t>>();
  auto temp_values = std::make_shared<gtsam::Values>();

  std::ifstream infile(filename);
  std::string line;
  while (std::getline(infile, line)) {
    std::stringstream ss(line);
    std::string tag;
    ss >> tag;
    if (tag == "NODE" || tag == "NODE_TEMP") {
      size_t key;
      double x, y, z, qx, qy, qz, qw;
      ss >> key >> x >> y >> z >> qx >> qy >> qz >> qw;
      gtsam::Pose3 pose(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      if (tag == "NODE") {
        values->insert(key, pose);
        char node_prefix = gtsam::Symbol(key).chr();
        if (initial_poses.count(node_prefix) == 0) {
          initial_poses[node_prefix] = std::vector<gtsam::Pose3>();
        }
        // Implicit assumption that node is in order
        initial_poses[node_prefix].push_back(pose);
      } else {
        temp_values->insert(key, pose);
        temp_initial_poses[key] = pose;
      }
    } else if (tag == "BETWEEN" || tag == "BETWEEN_TEMP") {
      size_t key1, key2;
      double x, y, z, qx, qy, qz, qw;
      gtsam::Matrix6 m;
      ss >> key1 >> key2 >> x >> y >> z >> qx >> qy >> qz >> qw;
      for (size_t i = 0; i < 6; i++) {
        for (size_t j = i; j < 6; j++) {
          double e_ij;
          ss >> e_ij;
          m(i, j) = e_ij;
          m(j, i) = e_ij;
        }
      }
      gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "BETWEEN") {
        nfg->add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, meas, noise));
      } else {
        temp_nfg->add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, meas, noise));
      }
    } else if (tag == "DEDGE" || tag == "DEDGE_TEMP") {
      size_t key1, key2;
      double x, y, z;
      gtsam::Matrix3 m;
      ss >> key1 >> key2 >> x >> y >> z;
      for (size_t i = 0; i < 3; i++) {
        for (size_t j = i; j < 3; j++) {
          double e_ij;
          ss >> e_ij;
          m(i, j) = e_ij;
          m(j, i) = e_ij;
        }
      }
      gtsam::Point3 measurement(x, y, z);
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "DEDGE") {
        nfg->add(DeformationEdgeFactor(key1, key2, measurement, noise));
      } else {
        temp_nfg->add(DeformationEdgeFactor(key1, key2, measurement, noise));
      }
    } else if (tag == "PRIOR") {
      size_t key;
      double x, y, z, qx, qy, qz, qw;
      gtsam::Matrix6 m;
      ss >> key >> x >> y >> z >> qx >> qy >> qz >> qw;
      for (size_t i = 0; i < 6; i++) {
        for (size_t j = i; j < 6; j++) {
          double e_ij;
          ss >> e_ij;
          m(i, j) = e_ij;
          m(j, i) = e_ij;
        }
      }

      gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      nfg->add(gtsam::PriorFactor<gtsam::Pose3>(key, meas, noise));
    } else if (tag == "KNOWN_INLIERS") {
      size_t idx;
      while (ss >> idx) {
        known_inliers->insert(idx);
      }
    } else if (tag == "TEMP_KNOWN_INLIERS") {
      size_t idx;
      while (ss >> idx) {
        temp_known_inliers->insert(idx);
      }
    } else if (tag == "VERTEX") {
      size_t key;
      double x, y, z;
      Timestamp n_sec;
      ss >> key >> n_sec >> x >> y >> z;
      gtsam::Symbol vertex_symb(key);
      char vertex_prefix = vertex_symb.chr();
      size_t vertex_index = vertex_symb.index();
      if (vertex_index == 0) {
        vertex_positions[vertex_prefix] = std::vector<gtsam::Point3>{};
        vertex_stamps[vertex_prefix] = std::vector<Timestamp>{};
      }
      assert(vertex_index == vertex_positions_[vertex_prefix].size());
      vertex_positions[vertex_prefix].push_back(gtsam::Point3(x, y, z));
      vertex_stamps[vertex_prefix].push_back(n_sec);
    } else {
      std::invalid_argument("DeformationGraph load: unknown tag. ");
    }
  }

  std::cout << "Loaded " << nfg->size() << " factors and " << temp_nfg->size()
            << " factors" << std::endl;
  return DeformationGraph::fromValues(values,
                                      nfg,
                                      known_inliers,
                                      temp_values,
                                      temp_nfg,
                                      temp_known_inliers,
                                      initial_poses,
                                      temp_initial_poses,
                                      vertex_positions,
                                      vertex_stamps);
}

}  // namespace kimera_pgmo

struct AppArgs {
  std::filesystem::path input;
  std::filesystem::path output;

  void add_args(CLI::App& app) {
    app.add_option("filepath", input)
        ->check(CLI::ExistingFile)
        ->required()
        ->description("Path to input dgraph file");
    app.add_option("--output", output, "Optional output file");
  }
};

auto main(int argc, char* argv[]) -> int {
  CLI::App app("Node publishing parent_T_child from CSV");
  app.allow_extras();
  app.get_formatter()->column_width(50);

  AppArgs args;
  args.add_args(app);
  try {
    app.parse(argc, argv);
  } catch (const CLI::ParseError& e) {
    return app.exit(e);
  }

  auto dgraph = kimera_pgmo::loadDeformationGraph(args.input);
  if (args.output.empty()) {
    args.output = args.input.replace_extension(".json");
  }

  dgraph->save(args.output);
  return 0;
}
