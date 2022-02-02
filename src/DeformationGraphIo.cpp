/**
 * @file   DeformationGraphIo.cpp
 * @brief  Deformation graph save and load functions
 * @author Yun Chang
 */
#include <fstream>
#include <stdexcept>

#include "kimera_pgmo/DeformationGraph.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

namespace kimera_pgmo {

void streamValue(const gtsam::Key& key,
                 const gtsam::Pose3& pose,
                 std::ofstream& stream) {
  const gtsam::Point3 t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  stream << key << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x()
         << " " << q.y() << " " << q.z() << " " << q.w();
}

void streamBetweenFactor(const gtsam::BetweenFactor<gtsam::Pose3>& between,
                         std::ofstream& stream) {
  std::string str;
  gtsam::SharedNoiseModel model = between.noiseModel();
  boost::shared_ptr<gtsam::noiseModel::Gaussian> gaussianModel =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(model);
  if (!gaussianModel) {
    model->print("model\n");
    throw std::invalid_argument("DeformationGraph save: invalid noise model!");
  }
  gtsam::Matrix6 Info = gaussianModel->R().transpose() * gaussianModel->R();
  const gtsam::Pose3 meas = between.measured();
  const gtsam::Point3 p = meas.translation();
  const auto q = meas.rotation().toQuaternion();
  stream << between.key1() << " " << between.key2() << " " << p.x() << " "
         << p.y() << " " << p.z() << " " << q.x() << " " << q.y() << " "
         << q.z() << " " << q.w();

  for (size_t i = 0; i < 6; i++) {
    for (size_t j = i; j < 6; j++) {
      stream << " " << Info(i, j);
    }
  }
}

void streamDedgeFactor(const DeformationEdgeFactor& dedge,
                       std::ofstream& stream) {
  std::string str;
  gtsam::SharedNoiseModel model = dedge.noiseModel();
  boost::shared_ptr<gtsam::noiseModel::Gaussian> gaussianModel =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(model);
  if (!gaussianModel) {
    model->print("model\n");
    throw std::invalid_argument("DeformationGraph save: invalid noise model. ");
  }
  gtsam::Matrix3 Info = gaussianModel->R().transpose() * gaussianModel->R();
  const gtsam::Pose3 from = dedge.fromPose();
  const gtsam::Point3 p = from.translation();
  const auto q = from.rotation().toQuaternion();
  const gtsam::Point3 to = dedge.toPoint();
  stream << dedge.key1() << " " << dedge.key2() << " " << p.x() << " " << p.y()
         << " " << p.z() << " " << q.x() << " " << q.y() << " " << q.z() << " "
         << q.w() << " " << to.x() << " " << to.y() << " " << to.z();

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
  boost::shared_ptr<gtsam::noiseModel::Gaussian> gaussianModel =
      boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(model);
  if (!gaussianModel) {
    model->print("model\n");
    throw std::invalid_argument("DeformationGraph save: invalid noise model. ");
  }
  gtsam::Matrix6 Info = gaussianModel->R().transpose() * gaussianModel->R();
  const gtsam::Pose3 meas = prior.prior();
  const gtsam::Point3 p = meas.translation();
  const auto q = meas.rotation().toQuaternion();
  stream << prior.key() << " " << p.x() << " " << p.y() << " " << p.z() << " "
         << q.x() << " " << q.y() << " " << q.z() << " " << q.w();

  for (size_t i = 0; i < 6; i++) {
    for (size_t j = i; j < 6; j++) {
      stream << " " << Info(i, j);
    }
  }
}

void streamVertices(const char& prefix,
                    const std::vector<gtsam::Point3>& positions,
                    const std::vector<ros::Time>& stamps,
                    std::ofstream& stream) {
  assert(positions.size() == stamps.size());
  for (size_t index = 0; index < positions.size(); index++) {
    gtsam::Key key = gtsam::Symbol(prefix, index);
    stream << "VERTEX " << key << " " << stamps[index].toNSec() << " "
           << positions[index].x() << " " << positions[index].y() << " "
           << positions[index].z() << std::endl;
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
    auto between =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
    if (between) {
      stream << "BETWEEN ";
      streamBetweenFactor(*between, stream);
      stream << std::endl;
    }

    auto dedge = boost::dynamic_pointer_cast<DeformationEdgeFactor>(factor);
    if (dedge) {
      stream << "DEDGE ";
      streamDedgeFactor(*dedge, stream);
      stream << std::endl;
    }

    auto prior =
        boost::dynamic_pointer_cast<gtsam::PriorFactor<gtsam::Pose3>>(factor);
    if (prior) {
      stream << "PRIOR ";
      streamPriorFactor(*prior, stream);
      stream << std::endl;
    }
  }

  // save temporary factors
  for (const auto& factor : temp_nfg_) {
    auto between =
        boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
    if (between) {
      stream << "BETWEEN_TEMP ";
      streamBetweenFactor(*between, stream);
      stream << std::endl;
    }

    auto dedge = boost::dynamic_pointer_cast<DeformationEdgeFactor>(factor);
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

// TODO(Yun) clean up / move to another file
void DeformationGraph::load(const std::string& filename) {
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
      gtsam::Pose3 pose(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
      if (tag == "NODE") {
        new_vals.insert(gtsam::Key(key), pose);
      } else {
        new_temp_vals.insert(gtsam::Key(key), pose);
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
      gtsam::SharedNoiseModel noise =
          gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "BETWEEN") {
        new_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
            gtsam::Key(key1), gtsam::Key(key2), meas, noise));
      } else {
        new_temp_factors.add(gtsam::BetweenFactor<gtsam::Pose3>(
            gtsam::Key(key1), gtsam::Key(key2), meas, noise));
      }
    } else if (tag == "DEDGE" || tag == "DEDGE_TEMP") {
      size_t key1, key2;
      double x, y, z, qx, qy, qz, qw, to_x, to_y, to_z;
      gtsam::Matrix3 m;
      ss >> key1 >> key2 >> x >> y >> z >> qx >> qy >> qz >> qw >> to_x >>
          to_y >> to_z;
      for (size_t i = 0; i < 3; i++) {
        for (size_t j = i; j < 3; j++) {
          double e_ij;
          ss >> e_ij;
          m(i, j) = e_ij;
          m(j, i) = e_ij;
        }
      }
      gtsam::Pose3 from_pose(gtsam::Rot3(qw, qx, qy, qz),
                             gtsam::Point3(x, y, z));
      gtsam::Point3 to_point(to_x, to_y, to_z);
      gtsam::SharedNoiseModel noise =
          gtsam::noiseModel::Gaussian::Information(m);
      if (tag == "DEDGE") {
        new_factors.add(DeformationEdgeFactor(
            gtsam::Key(key1), gtsam::Key(key2), from_pose, to_point, noise));
        consistency_factors_.add(DeformationEdgeFactor(
            gtsam::Key(key1), gtsam::Key(key2), from_pose, to_point, noise));
      } else {
        new_temp_factors.add(DeformationEdgeFactor(
            gtsam::Key(key1), gtsam::Key(key2), from_pose, to_point, noise));
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
      gtsam::SharedNoiseModel noise =
          gtsam::noiseModel::Gaussian::Information(m);
      new_factors.add(
          gtsam::PriorFactor<gtsam::Pose3>(gtsam::Key(key), meas, noise));
    } else if (tag == "VERTEX") {
      size_t key;
      double x, y, z, n_sec;
      ss >> key >> n_sec >> x >> y >> z;
      gtsam::Symbol node_symb(key);
      char node_prefix = node_symb.chr();
      size_t node_index = node_symb.index();
      if (node_index == 0) {
        vertex_positions_[node_prefix] = std::vector<gtsam::Point3>{};
        vertex_stamps_[node_prefix] = std::vector<ros::Time>{};
      }
      assert(node_index == vertex_positions_[node_prefix].size());
      vertex_positions_[node_prefix].push_back(gtsam::Point3(x, y, z));
      vertex_stamps_[node_prefix].push_back(ros::Time(n_sec * 1e-9));
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
