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

void to_json(json& record, const StampedPose& pose) {
  record["pose"] = pose.pose;
  record["timestamp_ns"] = pose.timestamp_ns;
}

void from_json(const json& record, StampedPose& pose) {
  record.at("pose").get_to(pose.pose);
  record.at("timestamp_ns").get_to(pose.timestamp_ns);
}

void to_json(json& record, const StampedPoint& point) {
  record["pos"] = point.pos;
  record["timestamp_ns"] = point.timestamp_ns;
}

void from_json(const json& record, StampedPoint& point) {
  record.at("pos").get_to(point.pos);
  record.at("timestamp_ns").get_to(point.timestamp_ns);
}

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
                          std::optional<size_t> new_robot_id) {
  auto key = record.at(name).get<gtsam::Key>();
  return new_robot_id ? rekey(key, *new_robot_id) : key;
}

using AdjacencyMap = std::map<gtsam::Key, std::set<gtsam::Key>>;
using Trajectories = std::map<char, std::vector<StampedPose>>;
using ControlPoints = std::map<char, std::vector<StampedPoint>>;

AdjacencyMap rekeyAdjacency(const AdjacencyMap& original, size_t robot_id) {
  std::map<gtsam::Key, std::set<gtsam::Key>> adjacency;
  for (const auto& [key, neighbors] : original) {
    std::set<gtsam::Key> new_neighbors;
    for (const auto& other : neighbors) {
      new_neighbors.insert(rekey(other, robot_id));
    }

    adjacency[rekey(key, robot_id)] = new_neighbors;
  }

  return adjacency;
}

ControlPoints rekeyControlPoints(const ControlPoints& original, size_t robot_id) {
  ControlPoints updated;
  for (const auto& [prefix, points] : original) {
    updated[kimera_pgmo::robot_id_to_vertex_prefix.at(robot_id)] = points;
  }

  return updated;
}

Trajectories rekeyInitialPoses(const Trajectories& original, size_t robot_id) {
  Trajectories updated;
  for (const auto& [prefix, trajectory] : original) {
    updated[kimera_pgmo::robot_id_to_prefix.at(robot_id)] = trajectory;
  }

  return updated;
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

void save_values(json& value_record, const gtsam::Values& values) {
  for (const auto& [key, value] : values) {
    auto& record = value_record.emplace_back();
    record["key"] = key;
    record["value"] = value.cast<gtsam::Pose3>();
  }
}

gtsam::SharedNoiseModel load_noise(const json& factor) {
  const auto info = factor.at("information").get<Eigen::MatrixXd>();
  return gtsam::noiseModel::Gaussian::Information(info);
}

void add_value(const json& record,
               gtsam::Values& values,
               std::optional<size_t> new_robot_id) {
  auto key = record.at("key").get<gtsam::Key>();
  if (new_robot_id) {
    key = rekey(key, *new_robot_id);
  }

  const auto pose = record.at("value").get<gtsam::Pose3>();
  values.insert(key, pose);
}

void add_factor(const json& factor,
                gtsam::NonlinearFactorGraph& graph,
                std::optional<size_t> new_robot_id,
                bool include_priors) {
  const auto type = factor.at("type").get<std::string>();
  if (type == "between") {
    const auto key1 = get_mapped_key(factor, "key1", new_robot_id);
    const auto key2 = get_mapped_key(factor, "key2", new_robot_id);
    const auto pose = factor.at("measurement").get<gtsam::Pose3>();
    const auto noise = load_noise(factor);
    graph.add(gtsam::BetweenFactor<gtsam::Pose3>(key1, key2, pose, noise));
    return;
  }

  if (type == "dedge") {
    const auto key1 = get_mapped_key(factor, "key1", new_robot_id);
    const auto key2 = get_mapped_key(factor, "key2", new_robot_id);
    const auto point = factor.at("measurement").get<gtsam::Point3>();
    const auto noise = load_noise(factor);
    graph.add(DeformationEdgeFactor(key1, key2, point, noise));
    return;
  }

  if (type == "prior") {
    if (!include_priors) {
      return;
    }

    const auto key = get_mapped_key(factor, "key", new_robot_id);
    const auto prior = factor.at("measurement").get<gtsam::Pose3>();
    const auto noise = load_noise(factor);
    graph.add(gtsam::PriorFactor<gtsam::Pose3>(key, prior, noise));
    return;
  }

  throw std::domain_error("unknown factor type '" + type + "'");
}

void DeformationGraph::save(const std::string& filename) const {
  json root;
  root["add_init_vertex_prior"] = add_init_vertex_prior_;
  root["num_loopclosures"] = num_loopclosures_;

  root["trajectories"] = json::object();
  for (const auto& [prefix, trajectory] : trajectories_) {
    root["trajectories"][std::string(1, prefix)] = trajectory;
  }

  root["control_points"] = json::object();
  for (const auto& [prefix, points] : control_points_) {
    root["control_points"][std::string(1, prefix)] = points;
  }

  root["factors"] = json::array();
  save_factor_graph(root["factors"], nfg_);
  root["values"] = json::array();
  save_values(root["values"], values_);
  root["known_inliers"] = known_inliers_;
  root["inlier_weights"] = inlier_weights_;

  root["adjacency"] = adjacency_map_;

  std::ofstream stream;
  stream.open(filename);
  stream << root << std::endl;
  stream.close();
}

void load_poses(const json& record, Trajectories& trajectories) {
  if (record.contains("trajectories")) {
    for (const auto& [key, data] : record.at("trajectories").items()) {
      const char prefix = key.at(0);
      auto iter = trajectories.emplace(prefix, std::vector<StampedPose>()).first;
      data.get_to(iter->second);
    }
  } else {
    std::map<char, std::vector<Timestamp>> stamp_map;
    std::map<char, std::vector<gtsam::Pose3>> pose_map;
    record.at("pg_stamps").get_to(stamp_map);
    record.at("pg_initial_poses").get_to(pose_map);
    for (const auto& [key, poses] : pose_map) {
      std::vector<Timestamp> curr_stamps;
      if (stamp_map.count(key)) {
        curr_stamps = stamp_map.at(key);
      }

      auto iter = trajectories.emplace(key, std::vector<StampedPose>()).first;
      for (size_t i = 0; i < poses.size(); ++i) {
        iter->second.push_back({i < curr_stamps.size() ? curr_stamps[i] : 0, poses[i]});
      }
    }
  }
}

void load_control_points(const json& record, ControlPoints& points) {
  if (record.contains("control_points")) {
    for (const auto& [key, data] : record.at("control_points").items()) {
      const char prefix = key.at(0);
      auto iter = points.emplace(prefix, std::vector<StampedPoint>()).first;
      data.get_to(iter->second);
    }
  } else {
    for (const auto& [key, data] : record.at("vertices").items()) {
      const char prefix = key.at(0);
      std::vector<Timestamp> stamps;
      std::vector<gtsam::Point3> pos;
      data.at("pos").get_to(pos);
      data.at("stamps").get_to(stamps);
      auto iter = points.emplace(prefix, std::vector<StampedPoint>()).first;
      for (size_t i = 0; i < pos.size(); ++i) {
        iter->second.push_back({i < stamps.size() ? stamps[i] : 0, pos[i]});
      }
    }
  }
}

DeformationGraph::Ptr DeformationGraph::load(const std::filesystem::path& filepath,
                                             bool include_priors,
                                             std::optional<size_t> new_id) {
  if (!std::filesystem::exists(filepath)) {
    SPARK_LOG(ERROR) << "Invalid deformation graph file " << filepath;
    return nullptr;
  }

  const auto ext = filepath.extension();
  if (ext != ".json" && ext != ".jsn") {
    SPARK_LOG(WARNING) << "Unknown extension for file " << filepath;
  }

  std::ifstream f(filepath);
  const auto data = json::parse(f);

  auto graph = std::make_shared<DeformationGraph>();

  // config fields
  data.at("add_init_vertex_prior").get_to(graph->add_init_vertex_prior_);
  data.at("num_loopclosures").get_to(graph->num_loopclosures_);

  // initial values
  load_poses(data, graph->trajectories_);
  load_control_points(data, graph->control_points_);

  // normal factor graph
  const auto inliers = data.at("known_inliers").get<std::set<size_t>>();
  const auto weights = data.at("inlier_weights").get<std::vector<double>>();
  for (const auto& record : data.at("values")) {
    add_value(record, graph->values_, new_id);
  }

  const auto& factors = data.at("factors");
  for (size_t i = 0; i < factors.size(); ++i) {
    const auto index = graph->nfg_.size();
    add_factor(factors.at(i), graph->nfg_, new_id, include_priors);
    if (graph->nfg_.size() == index) {
      continue;
    }

    if (inliers.count(i)) {
      graph->known_inliers_.insert(index);
    }

    if (i < weights.size()) {
      graph->inlier_weights_.push_back(weights[i]);
    }
  }

  // factor tracking
  data.at("adjacency").get_to(graph->adjacency_map_);

  // rekey non gtsam members
  if (new_id) {
    graph->adjacency_map_ = rekeyAdjacency(graph->adjacency_map_, *new_id);
    graph->trajectories_ = rekeyInitialPoses(graph->trajectories_, *new_id);
    graph->control_points_ = rekeyControlPoints(graph->control_points_, *new_id);
  }

  return graph;
}

}  // namespace kimera_pgmo
