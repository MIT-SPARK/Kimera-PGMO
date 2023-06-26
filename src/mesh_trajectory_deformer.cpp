/**
 * @file   KimeraPgmo.cpp
 * @brief  KimeraPgmo class: Main class and ROS interface
 * @author Yun Chang
 */
#include <chrono>
#include <cmath>

#include "kimera_pgmo/KimeraPgmoInterface.h"

namespace kimera_pgmo {

class MeshTrajectoryDeformer : public KimeraPgmoInterface {
 public:
  MeshTrajectoryDeformer() : optimized_mesh_(new pcl::PolygonMesh) {}

  ~MeshTrajectoryDeformer() {}

  bool initialize(const ros::NodeHandle& n) {
    if (!loadParameters(n)) {
      ROS_ERROR("KimeraPgmo: Failed to load parameters.");
    }
    pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
    kimera_pgmo::ReadMeshWithStampsFromPly(ply_path_, mesh, &mesh_vertex_stamps_);

    original_mesh =
        PolygonMeshToPgmoMeshMsg(robot_id_, *mesh, mesh_vertex_stamps_, "world");
    loadDeformationGraphFromFile(dgrf_path_, robot_id_);
    loadPoseGraphSparseMapping(sparse_mapping_path_);

    ROS_INFO("Load mesh and graph success");

    loadOriginalPath(orig_traj_path_);
    return loadOptimizedPath(optimized_traj_path_, max_diff_ns_);
  }

  bool save() {
    saveDeformationGraph(config_.log_path + "/optimized.dgrf");
    return saveMesh(*optimized_mesh_, config_.log_path + "/optimized.ply");
  }

  bool createPublishers(const ros::NodeHandle& n) { return true; }

  bool registerCallbacks(const ros::NodeHandle& n) { return true; }

 private:
  // Load deformation parameters
  bool loadParameters(const ros::NodeHandle& n) {
    if (!KimeraPgmoInterface::loadParameters(n)) return false;
    ROS_INFO("Loaded interface parameters");
    if (!n.getParam("dgrf_path", dgrf_path_)) return false;
    if (!n.getParam("ply_path", ply_path_)) return false;
    if (!n.getParam("sparse_mapping_path", sparse_mapping_path_)) return false;
    ROS_INFO_STREAM("Loading DGRF file: " << dgrf_path_
                                          << " and ply file: " << ply_path_);
    if (!n.getParam("original_traj", orig_traj_path_)) return false;
    if (!n.getParam("optimized_traj", optimized_traj_path_)) return false;
    ROS_INFO_STREAM("Deforming based on trajectory: " << optimized_traj_path_);
    int max_diff_ns_int;
    if (!n.getParam("max_diff_ns", max_diff_ns_int)) return false;
    max_diff_ns_ = static_cast<uint64_t>(max_diff_ns_int);
    int robot_id_int;
    if (!n.getParam("robot_id", robot_id_int)) return false;
    robot_id_ = static_cast<size_t>(robot_id_int);

    if (config_.log_path != "") {
      ROS_INFO_STREAM("Saving optimized data to: "
                      << config_.log_path << "/ mesh_pgmo.ply and traj_pgmo.csv");
    }
    return true;
  }

  void loadOriginalPath(const std::string& path) {
    std::ifstream infile(path);
    if (!infile.is_open()) {
      ROS_ERROR("Could not open original trajectory file");
      ros::shutdown();
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
      ROS_ERROR("Could not open optimized trajectory file");
      ros::shutdown();
    }

    // Scalars that will be filled
    uint64_t timestamp;
    uint32_t pose_idx;
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
        ROS_WARN("Looking for stamp before mission start. ");
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
      pose_idx = std::stoi(token);
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
          {gtsam::Symbol(GetRobotPrefix(robot_id_), pose_idx), optimized_pose});
    }
    infile.close();

    if (node_estimates.size() == 0) {
      return false;
    }

    deformation_graph_->removePriorsWithPrefix(GetRobotPrefix(robot_id_));
    deformation_graph_->addNodeMeasurements(node_estimates, config_.prior_variance);
    ROS_INFO("Optimizing full mesh...");
    optimizeFullMesh(original_mesh, optimized_mesh_, &mesh_vertex_stamps_, true);

    return true;
  }

 private:
  std::string dgrf_path_;
  std::string ply_path_;
  std::string orig_traj_path_;
  std::string optimized_traj_path_;
  std::string sparse_mapping_path_;
  size_t robot_id_;
  KimeraPgmoMesh original_mesh;
  pcl::PolygonMesh::Ptr optimized_mesh_;
  std::vector<ros::Time> mesh_vertex_stamps_;
  uint64_t max_diff_ns_;

  std::map<uint64_t, size_t> keyed_stamps_;
};
}  // namespace kimera_pgmo

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "mesh_trajectory_deformer");
  ros::NodeHandle n("~");

  kimera_pgmo::MeshTrajectoryDeformer deformer;
  if (deformer.initialize(n)) {
    deformer.save();
  }
  return EXIT_SUCCESS;
}
