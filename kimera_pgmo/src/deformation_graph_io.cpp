#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <kimera_rpgo/pose_graph_4dof.h>
#include <unistd.h>

#include <filesystem>
#include <fstream>

#include <Eigen/Cholesky>
#include <nlohmann/json.hpp>

#include "kimera_pgmo/deformation_edge_factor.h"
#include "kimera_pgmo/deformation_graph.h"
#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {
namespace {

using Json = nlohmann::json;
using Prior3 = gtsam::PriorFactor<gtsam::Pose3>;
using Prior4 = gtsam::PriorFactor<gtsam::Pose4DoF>;
using Between3 = gtsam::BetweenFactor<gtsam::Pose3>;
using Between4 = gtsam::BetweenFactor<gtsam::Pose4DoF>;

uint64_t readUnsigned(const Json& value) {
  const auto text = value.get<std::string>();
  if (text.empty() || text.find_first_not_of("0123456789") != std::string::npos) {
    throw std::invalid_argument("Expected an unsigned decimal string");
  }

  return std::stoull(text);
}

Json pointJson(const gtsam::Point3& point) {
  if (!point.allFinite()) {
    throw std::invalid_argument("Non-finite point");
  }

  return Json::array({point.x(), point.y(), point.z()});
}

gtsam::Point3 readPoint(const Json& value) {
  if (!value.is_array() || value.size() != 3) {
    throw std::invalid_argument("Expected three point coordinates");
  }

  gtsam::Point3 point(
      value.at(0).get<double>(), value.at(1).get<double>(), value.at(2).get<double>());
  if (!point.allFinite()) {
    throw std::invalid_argument("Non-finite point");
  }

  return point;
}

Json poseJson(const gtsam::Pose3& pose) {
  const auto q = pose.rotation().toQuaternion();
  if (!q.coeffs().allFinite()) {
    throw std::invalid_argument("Non-finite rotation");
  }

  return {{"position", pointJson(pose.translation())},
          {"quaternion", Json::array({q.w(), q.x(), q.y(), q.z()})}};
}

gtsam::Pose3 readPose(const Json& value) {
  const auto& rotation = value.at("quaternion");
  if (!rotation.is_array() || rotation.size() != 4) {
    throw std::invalid_argument("Expected a wxyz quaternion");
  }

  Eigen::Quaterniond q(rotation.at(0).get<double>(),
                       rotation.at(1).get<double>(),
                       rotation.at(2).get<double>(),
                       rotation.at(3).get<double>());
  if (!q.coeffs().allFinite() || std::abs(q.norm() - 1.0) > 1e-6) {
    throw std::invalid_argument("Invalid unit quaternion");
  }

  return gtsam::Pose3(gtsam::Rot3(q.normalized()), readPoint(value.at("position")));
}

Json noiseJson(const gtsam::SharedNoiseModel& noise) {
  const auto gaussian = dynamic_cast<const gtsam::noiseModel::Gaussian*>(noise.get());
  if (!gaussian || dynamic_cast<const gtsam::noiseModel::Constrained*>(noise.get())) {
    throw std::invalid_argument(
        "Checkpoint supports unconstrained Gaussian noise only");
  }

  const auto info = gaussian->information();
  if (!info.allFinite()) {
    throw std::invalid_argument("Non-finite information matrix");
  }

  Json rows = Json::array();
  for (int i = 0; i < info.rows(); ++i) {
    Json row = Json::array();
    for (int j = 0; j < info.cols(); ++j) {
      row.push_back(info(i, j));
    }

    rows.push_back(std::move(row));
  }

  return rows;
}

gtsam::SharedNoiseModel readNoise(const Json& record, size_t dimension) {
  const auto& rows = record.at("information");
  if (!rows.is_array() || rows.size() != dimension) {
    throw std::invalid_argument("Incorrect information matrix dimension");
  }

  gtsam::Matrix info(dimension, dimension);
  for (size_t i = 0; i < dimension; ++i) {
    if (!rows.at(i).is_array() || rows.at(i).size() != dimension) {
      throw std::invalid_argument("Incorrect information matrix row dimension");
    }

    for (size_t j = 0; j < dimension; ++j) {
      info(i, j) = rows.at(i).at(j).get<double>();
    }
  }

  if (!info.allFinite() || !info.isApprox(info.transpose(), 1e-10) ||
      Eigen::LLT<gtsam::Matrix>(info).info() != Eigen::Success) {
    throw std::invalid_argument(
        "Information matrix must be finite, symmetric, and positive definite");
  }

  return gtsam::noiseModel::Gaussian::Information(info);
}

template <typename Factor>
bool writeDeformation(const gtsam::NonlinearFactor& factor,
                      const char* type,
                      Json& record) {
  const auto typed = dynamic_cast<const Factor*>(&factor);
  if (!typed) {
    return false;
  }

  record["type"] = type;
  record["measurement"] = pointJson(typed->measurement());
  return true;
}

Json factorJson(const gtsam::NonlinearFactor& factor) {
  Json record;
  record["keys"] = Json::array();
  for (const auto key : factor.keys()) {
    record["keys"].push_back(std::to_string(key));
  }

  const auto noisy = dynamic_cast<const gtsam::NoiseModelFactor*>(&factor);
  if (!noisy) {
    throw std::invalid_argument("Unsupported checkpoint factor");
  }

  record["information"] = noiseJson(noisy->noiseModel());
  if (const auto prior = dynamic_cast<const Prior3*>(&factor)) {
    record["type"] = "prior3";
    record["measurement"] = poseJson(prior->prior());
  } else if (const auto prior = dynamic_cast<const Prior4*>(&factor)) {
    record["type"] = "prior4";
    record["measurement"] = poseJson(prior->prior().pose());
  } else if (const auto between = dynamic_cast<const Between3*>(&factor)) {
    record["type"] = "between3";
    record["measurement"] = poseJson(between->measured());
  } else if (const auto between = dynamic_cast<const Between4*>(&factor)) {
    record["type"] = "between4";
    record["measurement"] = poseJson(between->measured().pose());
  } else if (!writeDeformation<DeformationEdgeFactor>(
                 factor, "deformation33", record) &&
             !writeDeformation<DeformationEdgeFactorToPose4DoF>(
                 factor, "deformation34", record) &&
             !writeDeformation<DeformationEdgeFactorFromPose4DoF>(
                 factor, "deformation43", record) &&
             !writeDeformation<DeformationEdgeFactor4DoF>(
                 factor, "deformation44", record)) {
    throw std::invalid_argument("Unsupported checkpoint factor type");
  }

  return record;
}

gtsam::NonlinearFactor::shared_ptr readFactor(const Json& record,
                                              const std::vector<gtsam::Key>& keys) {
  const auto type = record.at("type").get<std::string>();
  const auto& measurement = record.at("measurement");
  const bool prior = type == "prior3" || type == "prior4";
  if (keys.size() != (prior ? 1u : 2u)) {
    throw std::invalid_argument("Incorrect factor arity");
  }

  if (type == "prior3") {
    return boost::make_shared<Prior3>(
        keys[0], readPose(measurement), readNoise(record, 6));
  }

  if (type == "prior4") {
    return boost::make_shared<Prior4>(
        keys[0], gtsam::Pose4DoF(readPose(measurement)), readNoise(record, 4));
  }

  if (type == "between3") {
    return boost::make_shared<Between3>(
        keys[0], keys[1], readPose(measurement), readNoise(record, 6));
  }

  if (type == "between4") {
    return boost::make_shared<kimera_rpgo::Pose4BetweenFactor>(
        keys[0], keys[1], gtsam::Pose4DoF(readPose(measurement)), readNoise(record, 4));
  }

  const auto point = readPoint(measurement);
  const auto noise = readNoise(record, 3);
  if (type == "deformation33") {
    return boost::make_shared<DeformationEdgeFactor>(keys[0], keys[1], point, noise);
  }

  if (type == "deformation34") {
    return boost::make_shared<DeformationEdgeFactorToPose4DoF>(
        keys[0], keys[1], point, noise);
  }

  if (type == "deformation43") {
    return boost::make_shared<DeformationEdgeFactorFromPose4DoF>(
        keys[0], keys[1], point, noise);
  }

  if (type == "deformation44") {
    return boost::make_shared<DeformationEdgeFactor4DoF>(
        keys[0], keys[1], point, noise);
  }

  throw std::invalid_argument("Unsupported checkpoint factor type: " + type);
}

Json readDocument(const std::string& filename) {
  std::ifstream stream(filename);
  if (!stream) {
    throw std::runtime_error("Cannot open deformation graph: " + filename);
  }

  stream >> std::ws;
  if (stream.peek() != '{') {
    throw std::invalid_argument(
        "Use upgrade_deformation_graph.py to convert legacy .dgrf files");
  }

  const auto root = Json::parse(stream);
  if (!root.contains("format") ||
      root.at("format") != "kimera_pgmo.deformation_graph" ||
      !root.contains("version") || !root.at("version").is_number_integer() ||
      root.at("version") != 1) {
    throw std::invalid_argument(
        "Unsupported deformation graph format/version; use "
        "upgrade_deformation_graph.py for legacy files");
  }

  return root;
}

void atomicWrite(const std::string& filename, const Json& root) {
  // mkstemp reserves a distinct sibling so simultaneous writers cannot share it.
  auto pattern = filename + ".XXXXXX";
  std::vector<char> buffer(pattern.begin(), pattern.end());
  buffer.push_back('\0');
  const auto fd = mkstemp(buffer.data());
  if (fd < 0) {
    throw std::runtime_error("Cannot create checkpoint temporary file: " + filename);
  }

  close(fd);
  const std::filesystem::path temporary(buffer.data());
  try {
    std::ofstream stream(temporary);
    stream.exceptions(std::ios::failbit | std::ios::badbit);
    stream << root.dump() << '\n';
    stream.close();
    std::filesystem::rename(temporary, filename);
  } catch (...) {
    std::error_code ignored;
    std::filesystem::remove(temporary, ignored);
    throw;
  }
}

}  // namespace

struct DeformationGraph::Archive {
  using Remap = std::function<gtsam::Key(const Json&)>;

  static void readNodes(DeformationGraph& candidate,
                        const Json& state,
                        bool temp,
                        const Remap& remap) {
    gtsam::Values updates;
    for (const auto& node : state.at("nodes")) {
      const gtsam::Key key = remap(node.at("key"));
      const gtsam::Symbol symbol(key);
      const auto original = readPose(node.at("original"));
      const auto role = node.at("role").get<std::string>();
      if (role != "mesh" && role != "pose") {
        throw std::invalid_argument("Invalid node role");
      }

      if (candidate.original_poses_.count(key)) {
        throw std::invalid_argument("Duplicate key or robot remapping collision");
      }

      if (role == "mesh") {
        if (temp || node.at("trajectory").get<bool>() ||
            !candidate.addNewMeshNode(
                key, original, readUnsigned(node.at("stamp_ns")))) {
          throw std::invalid_argument("Invalid mesh node sequence");
        }
      } else {
        candidate.insertValue(key, original, temp);
        if (!node.at("stamp_ns").is_null()) {
          candidate.setPoseTimestamp(key, readUnsigned(node.at("stamp_ns")));
        }

        if (temp) {
          candidate.temp_pg_initial_poses_.emplace(key, original);
        } else if (node.at("trajectory").get<bool>()) {
          auto& poses = candidate.pg_initial_poses_[symbol.chr()];
          if (symbol.index() != poses.size()) {
            throw std::invalid_argument("Invalid trajectory node sequence");
          }

          poses.push_back(original);
        }
      }

      const auto estimate = readPose(node.at("estimate"));
      if (candidate.isPose4(key)) {
        const auto expected = original.rotation().ypr();
        const auto actual = estimate.rotation().ypr();
        if ((expected.tail<2>() - actual.tail<2>()).norm() > 1e-8) {
          throw std::invalid_argument("4DoF estimate changed fixed roll/pitch");
        }
      }

      updates.insert(key, estimate);
    }

    candidate.updateState(updates, temp);
  }

  static void readFactors(DeformationGraph& candidate,
                          const Json& input,
                          bool temp,
                          bool include_priors,
                          const Remap& remap) {
    auto& state = temp ? candidate.temporary_ : candidate.permanent_;
    std::optional<bool> has_weights;
    for (const auto& record : input.at("factors")) {
      const auto type = record.at("type").get<std::string>();
      std::vector<gtsam::Key> keys;
      for (const auto& key : record.at("keys")) {
        keys.push_back(remap(key));
      }

      const auto factor = readFactor(record, keys);
      for (size_t i = 0; i < keys.size(); ++i) {
        const auto key = keys[i];
        if (!candidate.permanent_.values.exists(key) &&
            !(temp && candidate.temporary_.values.exists(key))) {
          throw std::invalid_argument("Factor references a missing node");
        }

        const bool expected4 =
            type == "prior4" || type == "between4" ||
            (type.rfind("deformation", 0) == 0 && type.at(11 + i) == '4');
        if (candidate.isPose4(key) != expected4) {
          throw std::invalid_argument("Factor endpoint type does not match node type");
        }
      }

      const bool weighted = !record.at("weight").is_null();
      if (has_weights && *has_weights != weighted) {
        throw std::invalid_argument("Partially populated optimization weights");
      }

      has_weights = weighted;
      double weight = 0.0;
      if (weighted) {
        weight = record.at("weight").get<double>();
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
          throw std::invalid_argument("Invalid inlier weight");
        }
      }

      if (!include_priors && (type == "prior3" || type == "prior4")) {
        continue;
      }

      if (record.at("known_inlier").get<bool>()) {
        state.known_inliers.insert(state.factors.size());
      }

      state.factors.add(factor);
      if (weighted) {
        state.inlier_weights.push_back(weight);
      }
    }
  }
};

void DeformationGraph::save(const std::string& filename) const {
  Json root = {{"format", "kimera_pgmo.deformation_graph"},
               {"version", 1},
               {"pose_mode", pose_mode_ == PoseMode::POSE3 ? "POSE3" : "POSE4DOF"},
               {"add_initial_vertex_prior", add_init_vertex_prior_}};
  const auto write_state = [&](const OptimizationState& state, bool temp) {
    Json result = {{"nodes", Json::array()}, {"factors", Json::array()}};
    for (const auto& entry : state.values) {
      const gtsam::Symbol symbol(entry.key);
      const bool mesh = mesh_keys_.count(entry.key);
      const auto poses = temp ? temp_values_.get() : values_.get();
      Json node = {{"key", std::to_string(entry.key)},
                   {"role", mesh ? "mesh" : "pose"},
                   {"original", poseJson(originalPose(entry.key))},
                   {"estimate", poseJson(poses->at<gtsam::Pose3>(entry.key))},
                   {"stamp_ns", nullptr},
                   {"trajectory", false}};
      if (mesh) {
        node["stamp_ns"] =
            std::to_string(vertex_stamps_.at(symbol.chr()).at(symbol.index()));
      } else {
        const auto stamp = pose_stamps_.find(entry.key);
        if (stamp != pose_stamps_.end()) {
          node["stamp_ns"] = std::to_string(stamp->second);
        }

        const auto trajectory = pg_initial_poses_.find(symbol.chr());
        node["trajectory"] = !temp && trajectory != pg_initial_poses_.end() &&
                             symbol.index() < trajectory->second.size();
      }

      result["nodes"].push_back(std::move(node));
    }

    if (!state.inlier_weights.empty() &&
        state.inlier_weights.size() != state.factors.size()) {
      throw std::logic_error("Factor weights are not aligned");
    }

    for (size_t i = 0; i < state.factors.size(); ++i) {
      if (!state.factors[i]) {
        throw std::logic_error("Null factor in deformation graph");
      }

      auto factor = factorJson(*state.factors[i]);
      factor["known_inlier"] = state.known_inliers.count(i) != 0;
      factor["weight"] = nullptr;
      if (!state.inlier_weights.empty()) {
        const auto weight = state.inlier_weights[i];
        if (!std::isfinite(weight) || weight < 0.0 || weight > 1.0) {
          throw std::logic_error("Invalid inlier weight");
        }

        factor["weight"] = weight;
      }

      result["factors"].push_back(std::move(factor));
    }

    return result;
  };
  root["permanent"] = write_state(permanent_, false);
  root["temporary"] = write_state(temporary_, true);
  atomicWrite(filename, root);
}

void DeformationGraph::load(const std::string& filename,
                            const GraphLoadOptions& options) {
  const auto root = readDocument(filename);
  const auto mode = root.at("pose_mode").get<std::string>();
  if (mode != "POSE3" && mode != "POSE4DOF") {
    throw std::invalid_argument("Invalid graph pose mode");
  }

  DeformationGraph candidate(root.at("add_initial_vertex_prior").get<bool>(),
                             mode == "POSE3" ? PoseMode::POSE3 : PoseMode::POSE4DOF);
  for (const auto& [source, target] : options.robot_id_remapping) {
    if (!robot_id_to_prefix.count(source) || !robot_id_to_prefix.count(target)) {
      throw std::invalid_argument("Robot mapping contains an unsupported robot ID");
    }
  }

  std::map<size_t, size_t> robot_sources;
  const auto remap = [&](const Json& value) -> gtsam::Key {
    const gtsam::Symbol key(readUnsigned(value));
    const auto pose_robot = robot_prefix_to_id.find(key.chr());
    const auto mesh_robot = vertex_prefix_to_id.find(key.chr());
    if (pose_robot == robot_prefix_to_id.end() &&
        mesh_robot == vertex_prefix_to_id.end()) {
      return key;
    }

    const bool mesh = mesh_robot != vertex_prefix_to_id.end();
    const auto robot = mesh ? mesh_robot->second : pose_robot->second;
    const auto target = options.robot_id_remapping.find(robot);
    const auto destination =
        target == options.robot_id_remapping.end() ? robot : target->second;
    const auto inserted = robot_sources.emplace(destination, robot);
    if (!inserted.second && inserted.first->second != robot) {
      throw std::invalid_argument("Robot remapping merges distinct robot IDs");
    }

    return gtsam::Symbol(
        mesh ? GetVertexPrefix(destination) : GetRobotPrefix(destination), key.index());
  };
  // Loading must not synthesize priors while reconstructing mesh nodes.
  candidate.add_init_vertex_prior_ = false;
  Archive::readNodes(candidate, root.at("permanent"), false, remap);
  if (options.include_temp) {
    Archive::readNodes(candidate, root.at("temporary"), true, remap);
  }

  candidate.add_init_vertex_prior_ = root.at("add_initial_vertex_prior").get<bool>();
  Archive::readFactors(
      candidate, root.at("permanent"), false, options.include_priors, remap);
  if (options.include_temp) {
    Archive::readFactors(
        candidate, root.at("temporary"), true, options.include_priors, remap);
  }

  candidate.rebuildBookkeeping();
  permanent_ = std::move(candidate.permanent_);
  temporary_ = std::move(candidate.temporary_);
  original_poses_ = std::move(candidate.original_poses_);
  mesh_keys_ = std::move(candidate.mesh_keys_);
  pose_stamps_ = std::move(candidate.pose_stamps_);
  pg_initial_poses_ = std::move(candidate.pg_initial_poses_);
  temp_pg_initial_poses_ = std::move(candidate.temp_pg_initial_poses_);
  vertex_positions_ = std::move(candidate.vertex_positions_);
  vertex_stamps_ = std::move(candidate.vertex_stamps_);
  pose_mode_ = candidate.pose_mode_;
  add_init_vertex_prior_ = candidate.add_init_vertex_prior_;
  refreshEstimates();
  rebuildBookkeeping();
}

void DeformationGraph::load(const std::string& filename,
                            bool include_temp,
                            bool set_robot_id,
                            size_t new_robot_id,
                            bool include_priors) {
  GraphLoadOptions options;
  options.include_temp = include_temp;
  options.include_priors = include_priors;
  if (set_robot_id) {
    const auto root = readDocument(filename);
    std::set<size_t> robots;
    const auto scan = [&](const Json& state) {
      for (const auto& node : state.at("nodes")) {
        const gtsam::Symbol key(readUnsigned(node.at("key")));
        if (robot_prefix_to_id.count(key.chr())) {
          robots.insert(robot_prefix_to_id.at(key.chr()));
        }

        if (vertex_prefix_to_id.count(key.chr())) {
          robots.insert(vertex_prefix_to_id.at(key.chr()));
        }
      }
    };
    scan(root.at("permanent"));
    if (include_temp) {
      scan(root.at("temporary"));
    }

    if (robots.size() > 1) {
      throw std::invalid_argument(
          "Cannot force a multi-robot graph to one robot ID; supply an explicit "
          "mapping");
    }

    if (!robots.empty()) {
      options.robot_id_remapping.emplace(*robots.begin(), new_robot_id);
    }
  }

  load(filename, options);
}

}  // namespace kimera_pgmo
