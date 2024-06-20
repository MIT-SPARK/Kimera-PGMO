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

#include "kimera_pgmo/deformation_graph.h"

namespace kimera_pgmo {

namespace {
template <typename T, typename Ptr>
const T* cast_to_ptr(const Ptr& ptr) {
  return dynamic_cast<const T*>(ptr.get());
}
}  // namespace

void streamValue(const gtsam::Key& key,
                 const gtsam::Pose3& pose,
                 std::ofstream& stream) {
  const gtsam::Point3 t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  stream << key << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " "
         << q.y() << " " << q.z() << " " << q.w();
}

void streamBetweenFactor(const gtsam::BetweenFactor<gtsam::Pose3>& between,
                         std::ofstream& stream) {
  std::string str;
  gtsam::SharedNoiseModel model = between.noiseModel();
  auto gaussianModel = cast_to_ptr<gtsam::noiseModel::Gaussian>(model);
  if (!gaussianModel) {
    model->print("model\n");
    throw std::invalid_argument("DeformationGraph save: invalid noise model!");
  }
  gtsam::Matrix6 Info = gaussianModel->R().transpose() * gaussianModel->R();
  const gtsam::Pose3 meas = between.measured();
  const gtsam::Point3 p = meas.translation();
  const auto q = meas.rotation().toQuaternion();
  stream << between.key1() << " " << between.key2() << " " << p.x() << " " << p.y()
         << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
         << q.w();

  for (size_t i = 0; i < 6; i++) {
    for (size_t j = i; j < 6; j++) {
      stream << " " << Info(i, j);
    }
  }
}

void streamDedgeFactor(const DeformationEdgeFactor& dedge, std::ofstream& stream) {
  std::string str;
  gtsam::SharedNoiseModel model = dedge.noiseModel();
  auto gaussianModel = cast_to_ptr<gtsam::noiseModel::Gaussian>(model);
  if (!gaussianModel) {
    model->print("model\n");
    throw std::invalid_argument("DeformationGraph save: invalid noise model. ");
  }
  gtsam::Matrix3 Info = gaussianModel->R().transpose() * gaussianModel->R();
  const gtsam::Pose3 from = dedge.fromPose();
  const gtsam::Point3 p = from.translation();
  const auto q = from.rotation().toQuaternion();
  const gtsam::Point3 to = dedge.toPoint();
  stream << dedge.key1() << " " << dedge.key2() << " " << p.x() << " " << p.y() << " "
         << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << " "
         << to.x() << " " << to.y() << " " << to.z();

  for (size_t i = 0; i < 3; i++) {
    for (size_t j = i; j < 3; j++) {
      stream << " " << Info(i, j);
    }
  }
}

void streamPriorFactor(const gtsam::PriorFactor<gtsam::Pose3>& prior,
                       std::ofstream& stream) {
  std::string str;
  gtsam::SharedNoiseModel model = prior.noiseModel();
  auto gaussianModel = cast_to_ptr<gtsam::noiseModel::Gaussian>(model);
  if (!gaussianModel) {
    model->print("model\n");
    throw std::invalid_argument("DeformationGraph save: invalid noise model. ");
  }
  gtsam::Matrix6 Info = gaussianModel->R().transpose() * gaussianModel->R();
  const gtsam::Pose3 meas = prior.prior();
  const gtsam::Point3 p = meas.translation();
  const auto q = meas.rotation().toQuaternion();
  stream << prior.key() << " " << p.x() << " " << p.y() << " " << p.z() << " " << q.x()
         << " " << q.y() << " " << q.z() << " " << q.w();

  for (size_t i = 0; i < 6; i++) {
    for (size_t j = i; j < 6; j++) {
      stream << " " << Info(i, j);
    }
  }
}

void streamVertices(const char& prefix,
                    const std::vector<gtsam::Point3>& positions,
                    const std::vector<Timestamp>& stamps,
                    std::ofstream& stream) {
  assert(positions.size() == stamps.size());
  for (size_t index = 0; index < positions.size(); index++) {
    gtsam::Key key = gtsam::Symbol(prefix, index);
    stream << "VERTEX " << key << " " << stamps[index] << " " << positions[index].x()
           << " " << positions[index].y() << " " << positions[index].z() << std::endl;
  }
}

void DeformationGraph::save(const std::string& filename) const {
  std::ofstream stream;
  stream.open(filename);
  // save values
  for (const auto& key_value : values_) {
    const gtsam::Pose3& pose = values_.at<gtsam::Pose3>(key_value.key);
    stream << "NODE ";
    streamValue(key_value.key, pose, stream);
    stream << std::endl;
  }
  // save temp values
  for (const auto& key_value : temp_values_) {
    const gtsam::Pose3& pose = temp_values_.at<gtsam::Pose3>(key_value.key);
    stream << "NODE_TEMP ";
    streamValue(key_value.key, pose, stream);
    stream << std::endl;
  }

  // save factors
  for (const auto& factor : nfg_) {
    auto between = cast_to_ptr<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
    if (between) {
      stream << "BETWEEN ";
      streamBetweenFactor(*between, stream);
      stream << std::endl;
    }

    auto dedge = cast_to_ptr<DeformationEdgeFactor>(factor);
    if (dedge) {
      stream << "DEDGE ";
      streamDedgeFactor(*dedge, stream);
      stream << std::endl;
    }

    auto prior = cast_to_ptr<gtsam::PriorFactor<gtsam::Pose3>>(factor);
    if (prior) {
      stream << "PRIOR ";
      streamPriorFactor(*prior, stream);
      stream << std::endl;
    }
  }

  // save temporary factors
  for (const auto& factor : temp_nfg_) {
    auto between = cast_to_ptr<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
    if (between) {
      stream << "BETWEEN_TEMP ";
      streamBetweenFactor(*between, stream);
      stream << std::endl;
    }

    auto dedge = cast_to_ptr<DeformationEdgeFactor>(factor);
    if (dedge) {
      stream << "DEDGE_TEMP ";
      streamDedgeFactor(*dedge, stream);
      stream << std::endl;
    }
  }

  // save the initial positions and timestamps of the mesh vertices
  for (const auto& pfx_vertices : vertex_positions_) {
    streamVertices(pfx_vertices.first,
                   pfx_vertices.second,
                   vertex_stamps_.at(pfx_vertices.first),
                   stream);
  }
  stream.close();
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

// TODO(Yun) clean up / move to another file
void DeformationGraph::load(const std::string& filename,
                            bool include_temp,
                            bool set_robot_id,
                            size_t new_robot_id) {
  gtsam::Values new_vals, new_temp_vals;
  gtsam::NonlinearFactorGraph new_factors, new_temp_factors;

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
        new_vals.insert(gtsam_key, pose);
        // TODO this is different from the initial pose before save
        char node_prefix = gtsam_key.chr();
        if (pg_initial_poses_.count(node_prefix) == 0) {
          pg_initial_poses_[node_prefix] = std::vector<gtsam::Pose3>();
        }
        // Implicit assumption that node is in order
        pg_initial_poses_[node_prefix].push_back(pose);
      } else if (include_temp) {
        new_temp_vals.insert(gtsam_key, pose);
        temp_pg_initial_poses_[gtsam_key] = pose;
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
        new_factors.add(
            gtsam::BetweenFactor<gtsam::Pose3>(gtsam_key1, gtsam_key2, meas, noise));
      } else if (include_temp) {
        new_temp_factors.add(
            gtsam::BetweenFactor<gtsam::Pose3>(gtsam_key1, gtsam_key2, meas, noise));
      }
    } else if (tag == "DEDGE" || tag == "DEDGE_TEMP") {
      size_t key1, key2;
      double x, y, z, qx, qy, qz, qw, to_x, to_y, to_z;
      gtsam::Matrix3 m;
      ss >> key1 >> key2 >> x >> y >> z >> qx >> qy >> qz >> qw >> to_x >> to_y >> to_z;
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
      gtsam::Pose3 from_pose(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      gtsam::Point3 to_point(to_x, to_y, to_z);
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "DEDGE") {
        new_factors.add(
            DeformationEdgeFactor(gtsam_key1, gtsam_key2, from_pose, to_point, noise));
        consistency_factors_.add(
            DeformationEdgeFactor(gtsam_key1, gtsam_key2, from_pose, to_point, noise));
      } else if (include_temp) {
        new_temp_factors.add(
            DeformationEdgeFactor(gtsam_key1, gtsam_key2, from_pose, to_point, noise));
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
      if (set_robot_id) {
        gtsam_key = rekey(gtsam_key, new_robot_id);
      }
      gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
      new_factors.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam_key, meas, noise));
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
        vertex_positions_[vertex_prefix] = std::vector<gtsam::Point3>{};
        vertex_stamps_[vertex_prefix] = std::vector<Timestamp>{};
      }
      assert(vertex_index == vertex_positions_[vertex_prefix].size());
      vertex_positions_[vertex_prefix].push_back(gtsam::Point3(x, y, z));
      vertex_stamps_[vertex_prefix].push_back(n_sec);
    } else {
      std::invalid_argument("DeformationGraph load: unknown tag. ");
    }
  }
  pgo_->updateTempFactorsValues(new_temp_factors, new_temp_vals);
  pgo_->update(new_factors, new_vals);
  values_ = pgo_->calculateEstimate();
  nfg_ = pgo_->getFactorsUnsafe();
  temp_nfg_ = pgo_->getTempFactorsUnsafe();
  temp_values_ = pgo_->getTempValues();
}

}  // namespace kimera_pgmo
