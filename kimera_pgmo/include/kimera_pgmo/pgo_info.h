#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <filesystem>
#include <map>
#include <set>
#include <vector>

namespace kimera_pgmo {

struct PGOInfo {
 public:
  gtsam::Values values;
  gtsam::NonlinearFactorGraph factors;
  std::set<size_t> known_inliers;
  std::vector<double> inlier_weights;

  void clear();
  void rekey(const std::map<char, char>& prefix_remapping);
  void forceRobotId(size_t robot_id);

  void save(std::ostream& out, bool is_temp = false) const;
  void save(const std::filesystem::path& filepath, bool is_temp = false) const;

  void load(std::istream& in, bool is_temp = false, bool include_priors = true);
  static std::shared_ptr<PGOInfo> load(const std::filesystem::path,
                                       bool is_temp = false,
                                       bool include_priors = true);

 private:
  void rekey(const std::function<gtsam::Symbol(gtsam::Symbol)>& remapping);
};

}  // namespace kimera_pgmo
