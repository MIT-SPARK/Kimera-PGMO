/**
 * @file   pgo_info.cpp
 * @brief  PGOInfo class
 * @author Yun Chang
 */
#include "kimera_pgmo/pgo_info.h"

#include <algorithm>
#include <cmath>
#include <numeric>

#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {
namespace {

gtsam::Symbol rekeyRobotId(gtsam::Symbol key, size_t robot_id) {
  const auto prefix = key.chr();
  if (robot_prefix_to_id.count(prefix)) {
    return gtsam::Symbol(GetRobotPrefix(robot_id), key.index());
  }

  if (vertex_prefix_to_id.count(prefix)) {
    return gtsam::Symbol(GetVertexPrefix(robot_id), key.index());
  }

  return key;
}

}  // namespace

void PGOInfo::clear() {
  values.clear();
  factors.resize(0);
  known_inliers.clear();
  inlier_weights.clear();
}

void PGOInfo::rekey(const std::map<char, char>& prefix_map) {
  rekey([&](gtsam::Symbol key) {
    const auto iter = prefix_map.find(key.chr());
    return iter != prefix_map.end() ? gtsam::Symbol(iter->second, key.index()) : key;
  });
}

void PGOInfo::rekey(const std::function<gtsam::Symbol(gtsam::Symbol)>& key_remap) {
  gtsam::Values new_values;
  std::map<gtsam::Key, gtsam::Key> remapping;
  for (const auto& key_value_pair : values) {
    gtsam::Symbol key = key_remap(key_value_pair.key);
    remapping[key_value_pair.key] = key;
    new_values.insert(key, key_value_pair.value);
  }

  values = new_values;
  factors = factors.rekey(remapping);
}

void PGOInfo::forceRobotId(size_t robot_id) {
  rekey([&](gtsam::Symbol key) { return rekeyRobotId(key, robot_id); });
}

}  // namespace kimera_pgmo
