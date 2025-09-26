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
#include "kimera_pgmo/deformation_graph_factor.h"

namespace kimera_pgmo {
namespace {

template <typename T, typename Ptr>
const T* cast_to_ptr(const Ptr& ptr) {
  return dynamic_cast<const T*>(ptr.get());
}

struct DgraphVertex {
  gtsam::Symbol key;
  Timestamp timestamp_ns;
  gtsam::Point3 pos;
};

struct TagInfo {
  std::string node;
  std::string between;
  std::string dedge;
  std::string prior;
  std::string inlier;

  static TagInfo temp() {
    return {
        "NODE_TEMP",
        "BETWEEN_TEMP",
        "DEDGE_TEMP",
        "PRIOR_TEMP",
        "TEMP_KNOWN_INLIERS",
    };
  }

  static TagInfo nominal() {
    return {
        "NODE",
        "BETWEEN",
        "DEDGE",
        "PRIOR",
        "KNOWN_INLIERS",
    };
  }
};

class Parser {
 public:
  using ParseFunction = std::function<void(std::istream&)>;
  void addCallback(const std::string& tag, const ParseFunction& callback) {
    tag_parsers_[tag] = callback;
  }

  void parse(std::istream& in) const {
    std::string line;
    while (std::getline(in, line)) {
      std::stringstream ss(line);
      std::string tag;
      ss >> tag;

      auto iter = tag_parsers_.find(tag);
      if (iter == tag_parsers_.end()) {
        continue;
      }

      iter->second(ss);
    }
  }

 private:
  std::map<std::string, ParseFunction> tag_parsers_;
};

void streamValue(const gtsam::Key& key,
                 const gtsam::Pose3& pose,
                 std::ostream& stream) {
  const gtsam::Point3 t = pose.translation();
  const auto q = pose.rotation().toQuaternion();
  stream << key << " " << t.x() << " " << t.y() << " " << t.z() << " " << q.x() << " "
         << q.y() << " " << q.z() << " " << q.w();
}

void streamBetweenFactor(const gtsam::BetweenFactor<gtsam::Pose3>& between,
                         std::ostream& stream) {
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

void streamDedgeFactor(const DeformationEdgeFactor& dedge, std::ostream& stream) {
  std::string str;
  gtsam::SharedNoiseModel model = dedge.noiseModel();
  auto gaussianModel = cast_to_ptr<gtsam::noiseModel::Gaussian>(model);
  if (!gaussianModel) {
    model->print("model\n");
    throw std::invalid_argument("DeformationGraph save: invalid noise model. ");
  }
  gtsam::Matrix3 Info = gaussianModel->R().transpose() * gaussianModel->R();
  const gtsam::Point3 measurement = dedge.measurement();
  stream << dedge.key1() << " " << dedge.key2() << " " << measurement.x() << " "
         << measurement.y() << " " << measurement.z();

  for (size_t i = 0; i < 3; i++) {
    for (size_t j = i; j < 3; j++) {
      stream << " " << Info(i, j);
    }
  }
}

void streamPriorFactor(const gtsam::PriorFactor<gtsam::Pose3>& prior,
                       std::ostream& stream) {
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
                    std::ostream& stream) {
  assert(positions.size() == stamps.size());
  for (size_t index = 0; index < positions.size(); index++) {
    gtsam::Key key = gtsam::Symbol(prefix, index);
    stream << "VERTEX " << key << " " << stamps[index] << " " << positions[index].x()
           << " " << positions[index].y() << " " << positions[index].z() << std::endl;
  }
}

void parseValue(std::istream& in, gtsam::Values& values) {
  size_t key;
  double x, y, z, qx, qy, qz, qw;
  in >> key >> x >> y >> z >> qx >> qy >> qz >> qw;

  const gtsam::Symbol gtsam_key(key);
  gtsam::Pose3 pose(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
  values.insert(gtsam_key, pose);
}

void parseBetween(std::istream& in, gtsam::NonlinearFactorGraph& factors) {
  size_t key1, key2;
  double x, y, z, qx, qy, qz, qw;
  gtsam::Matrix6 m;
  in >> key1 >> key2 >> x >> y >> z >> qx >> qy >> qz >> qw;
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = i; j < 6; j++) {
      double e_ij;
      in >> e_ij;
      m(i, j) = e_ij;
      m(j, i) = e_ij;
    }
  }

  const gtsam::Symbol gtsam_key1(key1);
  const gtsam::Symbol gtsam_key2(key2);
  gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
  factors.add(gtsam::BetweenFactor<gtsam::Pose3>(gtsam_key1, gtsam_key2, meas, noise));
}

void parseDedge(std::istream& in, gtsam::NonlinearFactorGraph& factors) {
  size_t key1, key2;
  double x, y, z;
  gtsam::Matrix3 m;
  in >> key1 >> key2 >> x >> y >> z;
  for (size_t i = 0; i < 3; i++) {
    for (size_t j = i; j < 3; j++) {
      double e_ij;
      in >> e_ij;
      m(i, j) = e_ij;
      m(j, i) = e_ij;
    }
  }

  const gtsam::Symbol gtsam_key1(key1);
  const gtsam::Symbol gtsam_key2(key2);
  gtsam::Point3 measurement(x, y, z);
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
  factors.add(DeformationEdgeFactor(gtsam_key1, gtsam_key2, measurement, noise));
}

void parsePrior(std::istream& in, gtsam::NonlinearFactorGraph& factors) {
  size_t key;
  double x, y, z, qx, qy, qz, qw;
  gtsam::Matrix6 m;
  in >> key >> x >> y >> z >> qx >> qy >> qz >> qw;
  for (size_t i = 0; i < 6; i++) {
    for (size_t j = i; j < 6; j++) {
      double e_ij;
      in >> e_ij;
      m(i, j) = e_ij;
      m(j, i) = e_ij;
    }
  }

  const gtsam::Symbol gtsam_key(key);
  gtsam::Pose3 meas(gtsam::Rot3(qw, qx, qy, qz), gtsam::Point3(x, y, z));
  gtsam::SharedNoiseModel noise = gtsam::noiseModel::Gaussian::Information(m);
  factors.add(gtsam::PriorFactor<gtsam::Pose3>(gtsam_key, meas, noise));
}

void parseVertex(std::istream& in, DgraphVertex& vertex) {
  size_t key;
  double x, y, z;
  in >> key >> vertex.timestamp_ns >> x >> y >> z;
  vertex.pos = gtsam::Point3(x, y, z);
  vertex.key = gtsam::Symbol(key);
}

void parseInliers(std::istream& in, std::set<size_t>& inliers) {
  size_t idx;
  while (in >> idx) {
    inliers.insert(idx);
  }
}

void setupParser(Parser& parser, PGOInfo& info, bool is_temp, bool include_priors) {
  const auto tags = is_temp ? TagInfo::temp() : TagInfo::nominal();
  parser.addCallback(tags.node, [&info, tags](std::istream& ss) {
    SPARK_LOG(DEBUG) << "Parsing " << tags.node;
    parseValue(ss, info.values);
  });
  parser.addCallback(tags.between, [&info, tags](std::istream& ss) {
    SPARK_LOG(DEBUG) << "Parsing " << tags.between;
    parseBetween(ss, info.factors);
  });
  parser.addCallback(tags.dedge, [&info, tags](std::istream& ss) {
    SPARK_LOG(DEBUG) << "Parsing " << tags.dedge;
    parseDedge(ss, info.factors);
  });
  parser.addCallback(tags.inlier, [&info, tags](std::istream& ss) {
    SPARK_LOG(DEBUG) << "Parsing " << tags.inlier;
    parseInliers(ss, info.known_inliers);
  });
  if (include_priors) {
    parser.addCallback(tags.prior, [&info, tags](std::istream& ss) {
      SPARK_LOG(DEBUG) << "Parsing " << tags.prior;
      parsePrior(ss, info.factors);
    });
  }
}

}  // namespace

void PGOInfo::save(std::ostream& out, bool is_temp) const {
  const auto tags = is_temp ? TagInfo::temp() : TagInfo::nominal();
  for (const auto& entry : values) {
    out << tags.node << " ";
    streamValue(entry.key, entry.value.cast<gtsam::Pose3>(), out);
    out << std::endl;
  }

  for (const auto& factor : factors) {
    auto between = cast_to_ptr<gtsam::BetweenFactor<gtsam::Pose3>>(factor);
    if (between) {
      out << tags.between << " ";
      streamBetweenFactor(*between, out);
      out << std::endl;
    }

    auto dedge = cast_to_ptr<DeformationEdgeFactor>(factor);
    if (dedge) {
      out << tags.dedge << " ";
      streamDedgeFactor(*dedge, out);
      out << std::endl;
    }

    auto prior = cast_to_ptr<gtsam::PriorFactor<gtsam::Pose3>>(factor);
    if (prior) {
      out << tags.prior << " ";
      streamPriorFactor(*prior, out);
      out << std::endl;
    }
  }

  out << tags.inlier;
  for (const auto& idx : known_inliers) {
    out << " " << idx;
  }
  out << std::endl;
}

void PGOInfo::load(std::istream& in, bool is_temp, bool include_priors) {
  Parser parser;
  setupParser(parser, *this, is_temp, include_priors);
  parser.parse(in);
}

void PGOInfo::save(const std::filesystem::path& filepath, bool is_temp) const {
  std::ofstream fout(filepath);
  save(fout, is_temp);
}

std::shared_ptr<PGOInfo> PGOInfo::load(const std::filesystem::path filepath,
                                       bool is_temp,
                                       bool include_priors) {
  auto info = std::make_shared<PGOInfo>();
  std::ifstream fin(filepath);
  info->load(fin, is_temp, include_priors);
  return info;
}

void DeformationGraph::save(const std::string& filename) const {
  std::ofstream stream(filename);
  info_.save(stream, false);
  temp_info_.save(stream, true);

  // save the initial positions and timestamps of the mesh vertices
  for (const auto& [prefix, positions] : vertex_positions_) {
    streamVertices(prefix, positions, vertex_stamps_.at(prefix), stream);
  }

  stream.close();
}

void DeformationGraph::load(const std::string& filename,
                            bool include_temp,
                            bool include_priors) {
  info_.clear();
  temp_info_.clear();

  Parser parser;
  setupParser(parser, info_, false, include_priors);
  if (include_temp) {
    setupParser(parser, temp_info_, true, include_priors);
  }

  parser.addCallback("VERTEX", [&](std::istream& in) {
    DgraphVertex vertex;
    parseVertex(in, vertex);

    const auto vertex_prefix = vertex.key.chr();
    const auto vertex_index = vertex.key.index();
    if (vertex_index == 0) {
      vertex_positions_[vertex_prefix] = std::vector<gtsam::Point3>{};
      vertex_stamps_[vertex_prefix] = std::vector<Timestamp>{};
    }

    if (vertex_index != vertex_positions_[vertex_prefix].size()) {
      std::stringstream ss;
      ss << "Misaligned vertex indices: " << vertex_index << " vs. "
         << vertex_positions_[vertex_prefix].size() << "!";
      throw std::runtime_error(ss.str());
    }

    vertex_positions_[vertex_prefix].push_back(vertex.pos);
    vertex_stamps_[vertex_prefix].push_back(vertex.timestamp_ns);
  });

  std::ifstream infile(filename);
  parser.parse(infile);

  // TODO(nathan) dump all values in the initial pose
}

}  // namespace kimera_pgmo
