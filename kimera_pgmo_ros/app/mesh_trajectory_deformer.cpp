/**
 * @file   kimera_pgmo.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include <config_utilities/config.h>
#include <config_utilities/parsing/ros.h>
#include <kimera_pgmo/kimera_pgmo_interface.h>
#include <kimera_pgmo/utils/logging.h>
#include <kimera_pgmo/utils/mesh_io.h>

#include <chrono>
#include <cmath>

#include "kimera_pgmo_ros/ros_log_sink.h"

namespace kimera_pgmo {

struct InputConfig {
  std::string dgrf_path;
  std::string ply_path;
  std::string orig_traj_path;
  std::string optimized_traj_path;
  std::string sparse_mapping_path;

  size_t robot_id = 0;
  uint64_t max_diff_ns = 0;
};

void declare_config(InputConfig& config) {
  using namespace config;
  name("InputConfig");
  field(config.dgrf_path, "dgrf_path");
  field(config.ply_path, "ply_path");
  field(config.orig_traj_path, "orig_traj_path");
  field(config.optimized_traj_path, "optimized_traj_path");
  field(config.sparse_mapping_path, "sparse_mapping_path");
  field(config.robot_id, "robot_id");
  field(config.max_diff_ns, "max_diff_ns");
}

class MeshTrajectoryDeformer : public KimeraPgmoInterface {
 public:
  explicit MeshTrajectoryDeformer(const InputConfig& config) : inputs(config) {}

  ~MeshTrajectoryDeformer() {}

  const InputConfig inputs;

  bool initialize(const KimeraPgmoConfig& config) override {
    if (!KimeraPgmoInterface::initialize(config)) {
      SPARK_LOG(ERROR) << "KimeraPgmo: Failed to initialize";
    }

    SPARK_LOG(INFO) << "Loading DGRF file: " << inputs.dgrf_path
                    << " and ply file: " << inputs.ply_path;
    SPARK_LOG(INFO) << "Deforming based on trajectory: " << inputs.optimized_traj_path;
    if (config_.log_path != "") {
      SPARK_LOG(INFO) << "Saving optimized data to: " << config_.log_path
                      << "/ mesh_pgmo.ply and traj_pgmo.csv";
    }

    ReadMeshWithStampsFromPly(inputs.ply_path, original_mesh_, &vertex_stamps_);
    loadDeformationGraphFromFile(inputs.dgrf_path, inputs.robot_id);
    loadPoseGraphSparseMapping(inputs.sparse_mapping_path);
    SPARK_LOG(INFO) << "Load mesh and graph success";

    loadOriginalPath(inputs.orig_traj_path);
    return loadOptimizedPath(inputs.optimized_traj_path, inputs.max_diff_ns);
  }

  bool save() {
    saveDeformationGraph(config_.log_path + "/optimized.dgrf");
    return saveMesh(optimized_mesh_, config_.log_path + "/optimized.ply");
  }

 private:
  void loadOriginalPath(const std::string& path) {
    std::ifstream infile(path);
    if (!infile.is_open()) {
      SPARK_LOG(ERROR) << "Could not open original trajectory file";
      return;
    }

    size_t idx = 0;
    uint64_t timestamp;

    std::string line;
    std::string token;
    // Skip first line (headers)
    std::getline(infile, line);

    // Iterate over remaining lines
    while (std::getline(infile, line)) {
      std::istringstream ss(line);

      std::getline(ss, token, ',');
      timestamp = std::stoull(token);
      keyed_stamps_.insert({timestamp, idx});
      idx++;
    }
    infile.close();
  }

  bool loadOptimizedPath(const std::string& path, uint64_t max_diff_ns) {
    std::ifstream infile(path);
    if (!infile.is_open()) {
      SPARK_LOG(ERROR) << "Could not open optimized trajectory file";
      return false;
    }

    // Scalars that will be filled
    uint64_t timestamp;
    double qx, qy, qz, qw;
    double tx, ty, tz;

    std::string line;
    std::string token;

    // Skip first line (headers)
    std::getline(infile, line);

    std::vector<std::pair<gtsam::Key, gtsam::Pose3>> node_estimates;

    // Iterate over remaining lines
    while (std::getline(infile, line)) {
      std::istringstream ss(line);

      std::getline(ss, token, ',');
      timestamp = std::stoull(token);

      // Look for closest timestamp
      uint64_t closest_stamp = 0;
      std::map<uint64_t, size_t>::iterator low, prev;
      low = keyed_stamps_.lower_bound(timestamp);
      if (low == keyed_stamps_.end()) {
        // nothing found, maybe use rbegin()
        SPARK_LOG(WARNING) << "Looking for stamp before mission start. ";
        continue;
      } else if (low == keyed_stamps_.begin()) {
        if ((low->first - timestamp) > max_diff_ns) {
          continue;
        }
        closest_stamp = low->first;
      } else {
        prev = std::prev(low);
        if ((timestamp - prev->first) < (low->first - timestamp)) {
          if ((timestamp - prev->first) > max_diff_ns) {
            continue;
          }
          closest_stamp = prev->first;
        } else {
          if ((low->first - timestamp) > max_diff_ns) {
            continue;
          }
          closest_stamp = low->first;
        }
      }

      // Make this more general?
      std::getline(ss, token, ',');
      std::stoi(token);
      std::getline(ss, token, ',');
      qx = std::stod(token);
      std::getline(ss, token, ',');
      qy = std::stod(token);
      std::getline(ss, token, ',');
      qz = std::stod(token);
      std::getline(ss, token, ',');
      qw = std::stod(token);

      std::getline(ss, token, ',');
      tx = std::stod(token);
      std::getline(ss, token, ',');
      ty = std::stod(token);
      std::getline(ss, token, ',');
      tz = std::stod(token);

      gtsam::Pose3 optimized_pose;
      optimized_pose =
          gtsam::Pose3(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(tx, ty, tz));

      size_t pose_idx = keyed_stamps_.at(closest_stamp);
      node_estimates.push_back(
          {gtsam::Symbol(GetRobotPrefix(inputs.robot_id), pose_idx), optimized_pose});
    }
    infile.close();

    if (node_estimates.size() == 0) {
      return false;
    }

    deformation_graph_->removePriorsWithPrefix(GetRobotPrefix(inputs.robot_id));
    deformation_graph_->addNodeMeasurements(node_estimates, config_.prior_variance);

    SPARK_LOG(INFO) << "Optimizing full mesh...";
    std::vector<int> indices;
    optimizeFullMesh(inputs.robot_id,
                     original_mesh_,
                     vertex_stamps_,
                     indices,
                     optimized_mesh_,
                     true);
    return true;
  }

 private:
  // input structures
  pcl::PolygonMesh original_mesh_;
  std::vector<Timestamp> vertex_stamps_;
  std::map<uint64_t, size_t> keyed_stamps_;
  // output structure
  pcl::PolygonMesh optimized_mesh_;
};

}  // namespace kimera_pgmo

auto main(int argc, char* argv[]) -> int {
  ros::NodeHandle n("~");
  logging::Logger::addSink("ros", std::make_shared<kimera_pgmo::RosLogSink>());

  const auto inputs = config::fromRos<kimera_pgmo::InputConfig>(n);
  kimera_pgmo::MeshTrajectoryDeformer deformer(inputs);

  const auto config = config::fromRos<kimera_pgmo::KimeraPgmoConfig>(n);
  if (!deformer.initialize(config)) {
    return EXIT_FAILURE;
  }

  deformer.save();
  return EXIT_SUCCESS;
}
