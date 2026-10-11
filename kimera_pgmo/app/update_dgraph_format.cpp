#include <gtsam/slam/BetweenFactor.h>

#include <CLI/CLI.hpp>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph.h"

namespace kimera_pgmo {

struct DGRFLoader {
  static DeformationGraph::Ptr load(const std::filesystem::path& filename);
};

DeformationGraph::Ptr DGRFLoader::load(const std::filesystem::path& filename) {
  auto graph = std::make_shared<DeformationGraph>();

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
      gtsam::Symbol gtsam_key(key);
      gtsam::Pose3 pose(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      if (tag == "NODE") {
        graph->values_.insert(gtsam_key, pose);
        // TODO this is different from the initial pose before save
        char node_prefix = gtsam_key.chr();
        if (graph->pg_initial_poses_.count(node_prefix) == 0) {
          graph->pg_initial_poses_[node_prefix] = std::vector<gtsam::Pose3>();
        }
        // Implicit assumption that node is in order
        graph->pg_initial_poses_[node_prefix].push_back(pose);
      } else {
        graph->temp_values_.insert(gtsam_key, pose);
        graph->temp_pg_initial_poses_[gtsam_key] = pose;
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
      gtsam::Symbol gtsam_key1(key1);
      gtsam::Symbol gtsam_key2(key2);
      gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "BETWEEN") {
        graph->nfg_.add(
            gtsam::BetweenFactor<gtsam::Pose3>(gtsam_key1, gtsam_key2, meas, noise));
      } else {
        graph->temp_nfg_.add(
            gtsam::BetweenFactor<gtsam::Pose3>(gtsam_key1, gtsam_key2, meas, noise));
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
      gtsam::Symbol gtsam_key1(key1);
      gtsam::Symbol gtsam_key2(key2);
      gtsam::Point3 measurement(x, y, z);
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "DEDGE") {
        graph->nfg_.add(
            DeformationEdgeFactor(gtsam_key1, gtsam_key2, measurement, noise));
      } else {
        graph->temp_nfg_.add(
            DeformationEdgeFactor(gtsam_key1, gtsam_key2, measurement, noise));
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

      gtsam::Symbol gtsam_key(key);
      gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      graph->nfg_.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam_key, meas, noise));
    } else if (tag == "KNOWN_INLIERS") {
      size_t idx;
      while (ss >> idx) {
        graph->known_inliers_.insert(idx);
      }
    } else if (tag == "TEMP_KNOWN_INLIERS") {
      size_t idx;
      while (ss >> idx) {
        graph->temp_known_inliers_.insert(idx);
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
        graph->vertex_positions_[vertex_prefix] = std::vector<gtsam::Point3>{};
        graph->vertex_stamps_[vertex_prefix] = std::vector<Timestamp>{};
      }

      graph->vertex_positions_[vertex_prefix].push_back(gtsam::Point3(x, y, z));
      graph->vertex_stamps_[vertex_prefix].push_back(n_sec);
    } else {
      std::invalid_argument("DeformationGraph load: unknown tag. ");
    }
  }

  return graph;
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

  auto dgraph = kimera_pgmo::DGRFLoader::load(args.input);
  if (args.output.empty()) {
    args.output = args.input.replace_extension(".json");
  }

  dgraph->save(args.output);
  return 0;
}
