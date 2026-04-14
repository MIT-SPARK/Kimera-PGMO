/**
 * @file   deformation_graph_io.cpp
 * @brief  Deformation graph save and load functions
 * @author Yun Chang
 */
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

#include <filesystem>
#include <fstream>
#include <memory>
#include <stdexcept>

#include <nlohmann/json.hpp>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/utils/common_functions.h"
#include "kimera_pgmo/utils/logging.h"

using nlohmann::json;
namespace fs = std::filesystem;

namespace nlohmann {

template <typename Scalar, int Rows, int Cols>
struct adl_serializer<Eigen::Matrix<Scalar, Rows, Cols>> {
  static void to_json(json& j, const Eigen::Matrix<Scalar, Rows, Cols>& mat) {
    json* vec = &j;
    if (Rows == Eigen::Dynamic && Cols == Eigen::Dynamic) {
      j["rows"] = mat.rows();
      j["cols"] = mat.cols();
      vec = &j["data"];
    }

    for (int r = 0; r < mat.rows(); ++r) {
      for (int c = 0; c < mat.cols(); ++c) {
        vec->push_back(mat(r, c));
      }
    }
  }

  static void from_json(const json& j, Eigen::Matrix<Scalar, Rows, Cols>& mat) {
    const auto* vec = &j;
    int rows = Rows;
    int cols = Cols;
    if (Rows == Eigen::Dynamic && Cols == Eigen::Dynamic) {
      rows = j.at("rows").get<int>();
      cols = j.at("cols").get<int>();
      vec = &(j.at("data"));
    } else if (Rows == Eigen::Dynamic) {
      rows = j.size() / Cols;
    } else if (Cols == Eigen::Dynamic) {
      cols = j.size() / Rows;
    }

    if (vec->size() != static_cast<size_t>(rows * cols)) {
      std::stringstream ss;
      ss << "cannot decode matrix: [" << rows << ", " << cols << "] from "
         << vec->size() << " values";
      throw std::runtime_error(ss.str());
    }

    mat = Eigen::Matrix<Scalar, Rows, Cols>::Zero(rows, cols);
    for (size_t i = 0; i < vec->size(); ++i) {
      int r = i / cols;
      int c = i % cols;
      const auto& value = vec->at(i);
      mat(r, c) = value.is_null() ? std::numeric_limits<Scalar>::quiet_NaN()
                                  : value.get<Scalar>();
    }
  }
};

}  // namespace nlohmann

namespace gtsam {

void to_json(json& record, const Pose3& pose) {
  const auto q = pose.rotation().toQuaternion();
  record["x"] = pose.translation().x();
  record["y"] = pose.translation().y();
  record["z"] = pose.translation().z();
  record["qw"] = q.w();
  record["qx"] = q.x();
  record["qy"] = q.y();
  record["qz"] = q.z();
}

void from_json(const json& record, Pose3& pose) {
  pose = Pose3(Quaternion(record.at("qw").get<float>(),
                          record.at("qx").get<double>(),
                          record.at("qy").get<double>(),
                          record.at("qz").get<double>()),
               Eigen::Vector3d(record.at("x").get<double>(),
                               record.at("y").get<double>(),
                               record.at("z").get<double>())

  );
}

}  // namespace gtsam

namespace kimera_pgmo {
namespace {

template <typename T, typename Ptr>
const T* cast_to_ptr(const Ptr& ptr) {
  return dynamic_cast<const T*>(ptr.get());
}

gtsam::Key rekey(gtsam::Symbol key, size_t robot_id) {
  char new_prefix = kimera_pgmo::robot_id_to_prefix.at(robot_id);
  char new_vertex_prefix = kimera_pgmo::robot_id_to_vertex_prefix.at(robot_id);
  if (kimera_pgmo::robot_prefix_to_id.count(key.chr())) {
    return gtsam::Symbol(new_prefix, key.index());
  }

  if (kimera_pgmo::vertex_prefix_to_id.count(key.chr())) {
    return gtsam::Symbol(new_vertex_prefix, key.index());
  }

  return key;
}

gtsam::Key get_mapped_key(const json& record,
                          const std::string& name,
                          bool set_robot_id,
                          size_t new_robot_id) {
  auto key = record.at(name).get<gtsam::Key>();
  return set_robot_id ? rekey(key, new_robot_id) : key;
}

}  // namespace

template <typename FactorT>
void save_noise(json& record, const FactorT& factor) {
  const auto model = factor.noiseModel();
  const auto gaussian = cast_to_ptr<gtsam::noiseModel::Gaussian>(model);
  if (!gaussian) {
    model->print("model\n");
    throw std::invalid_argument("invalid noise model!");
  }

  record["information"] = gaussian->information();
}

void save_factor(json& record, const gtsam::BetweenFactor<gtsam::Pose3>& between) {
  save_noise(record, between);
  record["type"] = "between";
  record["key1"] = between.key1();
  record["key2"] = between.key2();
  record["measurement"] = between.measured();
}

void save_factor(json& record, const DeformationEdgeFactor& dedge) {
  save_noise(record, dedge);
  record["type"] = "dedge";
  record["key1"] = dedge.key1();
  record["key2"] = dedge.key2();
  record["measurement"] = dedge.measurement();
}

void save_factor(json& record, const gtsam::PriorFactor<gtsam::Pose3>& prior) {
  save_noise(record, prior);
  record["type"] = "prior";
  record["key"] = prior.key();
  record["measurement"] = prior.prior();
}

void save_factor_graph(json& graph_record, const gtsam::NonlinearFactorGraph& graph) {
  for (const auto& factor : graph) {
    if (!factor) {
      continue;
    }

    auto& record = graph_record.emplace_back();
    const auto between = cast_to_ptr<const gtsam::BetweenFactor<gtsam::Pose3>>(factor);
    if (between) {
      save_factor(record, *between);
      continue;
    }

    const auto dedge = cast_to_ptr<const DeformationEdgeFactor>(factor);
    if (dedge) {
      save_factor(record, *dedge);
      continue;
    }

    const auto prior = cast_to_ptr<const gtsam::PriorFactor<gtsam::Pose3>>(factor);
    if (prior) {
      save_factor(record, *prior);
      continue;
    }

    factor->print("factor\n");
    throw std::invalid_argument("unknown factor type!");
  }
}

gtsam::SharedNoiseModel load_noise(const json& factor) {
  const auto info = factor.at("information").get<Eigen::MatrixXd>();
  return gtsam::noiseModel::Gaussian::Information(info);
}

void add_factor(const json& factor,
                gtsam::NonlinearFactorGraph& graph,
                bool set_robot_id,
                size_t new_robot_id,
                bool include_priors) {
  const auto type = factor.at("type").get<std::string>();
  if (type == "between") {
    const auto key1 = get_mapped_key(factor, "key1", set_robot_id, new_robot_id);
    const auto key2 = get_mapped_key(factor, "key2", set_robot_id, new_robot_id);
    const auto pose = factor.at("measurement").get<gtsam::Pose3>();
    const auto noise = load_noise(factor);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, pose, noise));
    return;
  }

  if (type == "dedge") {
    const auto key1 = get_mapped_key(factor, "key1", set_robot_id, new_robot_id);
    const auto key2 = get_mapped_key(factor, "key2", set_robot_id, new_robot_id);
    const auto point = factor.at("measurement").get<gtsam::Point3>();
    const auto noise = load_noise(factor);
    graph.add(DeformationEdgeFactor(key1, key2, point, noise));
    return;
  }

  if (type == "prior") {
    if (!include_priors) {
      return;
    }

    const auto key = get_mapped_key(factor, "key", set_robot_id, new_robot_id);
    const auto prior = factor.at("measurement").get<gtsam::Pose3>();
    const auto noise = load_noise(factor);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(key, prior, noise));
    return;
  }

  throw std::domain_error("unknown factor type '" + type + "'");
}

void DeformationGraph::save(const std::string& filename) const {
  json root;
  root["mesh_only"] = add_init_vertex_prior_;

  root["values"] = json::array();
  for (const auto& [key, value] : *values_) {
    auto& record = root["values"].emplace_back();
    record["key"] = key;
    record["value"] = value.cast<gtsam::Pose3>();
  }

  root["factors"] = json::array();
  save_factor_graph(root["factors"], *nfg_);
  root["known_inliers"] = *known_inliers_;
  root["inlier_weights"] = *inlier_weights_;

  root["temp_values"] = json::array();
  for (const auto& [key, value] : *temp_values_) {
    auto& record = root["temp_values"].emplace_back();
    record["key"] = key;
    record["value"] = value.cast<gtsam::Pose3>();
  }

  root["temp_factors"] = json::array();
  save_factor_graph(root["temp_factors"], *temp_nfg_);
  root["temp_known_inliers"] = *temp_known_inliers_;

  // save the initial positions and timestamps of the mesh vertices
  root["vertices"] = json::object();
  for (const auto& [prefix, vertices] : vertex_positions_) {
    auto& record = root["vertices"][std::string(1, prefix)];
    record["pos"] = vertices;
    record["stamps"] = vertex_stamps_.at(prefix);
  }

  root["inlier_weights"] = *inlier_weights_;
  root["temp_inlier_weights"] = *temp_inlier_weights_;
  root["adjacency"] = adjacency_map_;

  std::ofstream stream;
  stream.open(filename);
  stream << root << std::endl;
  stream.close();
}

DeformationGraph::Ptr DeformationGraph::load(const fs::path& filepath,
                                             bool include_temp,
                                             bool set_id,
                                             size_t new_id,
                                             bool include_priors) {
  if (!std::filesystem::exists(filepath)) {
    SPARK_LOG(ERROR) << "Invalid deformation graph file " << filepath;
    return nullptr;
  }

  const auto ext = filepath.extension();
  if (ext == ".json" || ext == ".jsn") {
    return loadFromJson(filepath, include_temp, set_id, new_id, include_priors);
  }

  if (ext == ".dgrf") {
    SPARK_LOG(WARNING) << "DGRF format deprecated!";
    return loadFromDgrf(filepath, include_temp, set_id, new_id, include_priors);
  }

  SPARK_LOG(ERROR) << "Unknown extension for file " << filepath;
  return nullptr;
}

DeformationGraph::Ptr DeformationGraph::loadFromJson(const fs::path& filepath,
                                                     bool include_temp,
                                                     bool set_robot_id,
                                                     size_t new_robot_id,
                                                     bool include_priors) {
  std::ifstream f(filepath);
  const auto data = json::parse(f);
  auto graph = std::make_shared<DeformationGraph>(data.at("mesh_only").get<bool>());

  for (const auto& record : data.at("values")) {
    auto key = record.at("key").get<gtsam::Key>();
    if (set_robot_id) {
      key = rekey(key, new_robot_id);
    }

    const auto pose = record.at("value").get<gtsam::Pose3>();
    graph->values_->insert(key, pose);
    // TODO(nathan) this is different from the initial pose before save
    char node_prefix = gtsam::Symbol(key).chr();
    if (graph->pg_initial_poses_.count(node_prefix) == 0) {
      graph->pg_initial_poses_[node_prefix] = std::vector<gtsam::Pose3>();
    }

    graph->pg_initial_poses_[node_prefix].push_back(pose);
  }

  for (const auto& factor : data.at("factors")) {
    add_factor(factor, *graph->nfg_, set_robot_id, new_robot_id, include_priors);
  }

  data.at("known_inliers").get_to(*graph->known_inliers_);
  data.at("inlier_weights").get_to(*graph->inlier_weights_);
  data.at("adjacency").get_to(graph->adjacency_map_);

  for (const auto& [prefix, record] : data.at("vertices").items()) {
    char vertex_prefix = prefix.at(0);
    if (set_robot_id && kimera_pgmo::vertex_prefix_to_id.count(vertex_prefix) > 0) {
      vertex_prefix = kimera_pgmo::robot_id_to_vertex_prefix.at(new_robot_id);
    }

    record.at("pos").get_to(graph->vertex_positions_[vertex_prefix]);
    record.at("stamps").get_to(graph->vertex_stamps_[vertex_prefix]);
  }

  if (!include_temp) {
    return graph;
  }

  for (const auto& record : data.at("temp_values")) {
    auto key = record.at("key").get<gtsam::Key>();
    if (set_robot_id) {
      key = rekey(key, new_robot_id);
    }

    const auto pose = record.at("value").get<gtsam::Pose3>();
    graph->temp_values_->insert(key, pose);
    graph->temp_pg_initial_poses_[key] = pose;
  }

  for (const auto& factor : data.at("temp_factors")) {
    add_factor(factor, *graph->temp_nfg_, set_robot_id, new_robot_id, include_priors);
  }

  data.at("temp_inlier_weights").get_to(*graph->temp_inlier_weights_);
  data.at("temp_known_inliers").get_to(*graph->temp_known_inliers_);
  return graph;
}

DeformationGraph::Ptr DeformationGraph::loadFromDgrf(
    const std::filesystem::path& filename,
    bool include_temp,
    bool set_robot_id,
    size_t new_robot_id,
    bool include_priors) {
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
      if (set_robot_id) {
        gtsam_key = rekey(gtsam_key, new_robot_id);
      }
      gtsam::Pose3 pose(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      if (tag == "NODE") {
        graph->values_->insert(gtsam_key, pose);
        // TODO this is different from the initial pose before save
        char node_prefix = gtsam_key.chr();
        if (graph->pg_initial_poses_.count(node_prefix) == 0) {
          graph->pg_initial_poses_[node_prefix] = std::vector<gtsam::Pose3>();
        }
        // Implicit assumption that node is in order
        graph->pg_initial_poses_[node_prefix].push_back(pose);
      } else if (include_temp) {
        graph->temp_values_->insert(gtsam_key, pose);
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
      if (set_robot_id) {
        gtsam_key1 = rekey(gtsam_key1, new_robot_id);
        gtsam_key2 = rekey(gtsam_key2, new_robot_id);
      }
      gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "BETWEEN") {
        graph->nfg_->add(
            gtsam::BetweenFactor<gtsam::Pose3>(gtsam_key1, gtsam_key2, meas, noise));
      } else if (include_temp) {
        graph->temp_nfg_->add(
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
      if (set_robot_id) {
        gtsam_key1 = rekey(key1, new_robot_id);
        gtsam_key2 = rekey(key2, new_robot_id);
      }
      gtsam::Point3 measurement(x, y, z);
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "DEDGE") {
        graph->nfg_->add(
            DeformationEdgeFactor(gtsam_key1, gtsam_key2, measurement, noise));
      } else if (include_temp) {
        graph->temp_nfg_->add(
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

      if (include_priors) {
        gtsam::Symbol gtsam_key(key);
        if (set_robot_id) {
          gtsam_key = rekey(gtsam_key, new_robot_id);
        }

        gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
        gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
        graph->nfg_->add(gtsam::PriorFactor<gtsam::Pose3>(gtsam_key, meas, noise));
      }
    } else if (tag == "KNOWN_INLIERS") {
      size_t idx;
      while (ss >> idx) {
        graph->known_inliers_->insert(idx);
      }
    } else if (tag == "TEMP_KNOWN_INLIERS") {
      size_t idx;
      while (ss >> idx) {
        graph->temp_known_inliers_->insert(idx);
      }
    } else if (tag == "VERTEX") {
      size_t key;
      double x, y, z;
      Timestamp n_sec;
      ss >> key >> n_sec >> x >> y >> z;
      gtsam::Symbol vertex_symb(key);
      char vertex_prefix = vertex_symb.chr();
      if (set_robot_id && kimera_pgmo::vertex_prefix_to_id.count(vertex_prefix) > 0) {
        vertex_prefix = kimera_pgmo::robot_id_to_vertex_prefix.at(new_robot_id);
      }
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
