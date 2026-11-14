/**
 * @file   common_functions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

bool isRobotPrefix(char prefix) { return prefix >= 'a' && prefix <= 'r'; }

char GetRobotPrefix(size_t robot_id) {
  if (!robot_id_to_prefix.count(robot_id)) {
    return '\0';
  }

  return robot_id_to_prefix.at(robot_id);
}

bool isVertexPrefix(char prefix) {
  return (prefix >= 's' && prefix <= 'z') || (prefix >= 'A' && prefix <= 'J');
}

bool isVertexKey(gtsam::Key key) { return isVertexPrefix(gtsam::Symbol(key).chr()); }

char getVertexPrefix(size_t robot_id) {
  if (robot_id < 8) {
    return 's' + static_cast<char>(robot_id);
  } else if (robot_id < 19) {
    return 'A' + static_cast<char>(robot_id - 8);
  }

  throw std::domain_error("only 19 robot IDs are supported currently");
}

size_t idFromVertexPrefix(char prefix) {
  if (prefix >= 's' && prefix <= 'z') {
    return static_cast<size_t>(prefix - 's');
  }

  if (prefix >= 'A' && prefix <= 'J') {
    return static_cast<size_t>(prefix - 'A') + 8;
  }

  throw std::domain_error("only 19 robot IDs are supported currently");
}

size_t idFromVertexKey(gtsam::Key key) {
  return idFromVertexPrefix(gtsam::Symbol(key).chr());
}

bool IsMeshVertex(gtsam::Key key) { return isVertexKey(key); }

traits::Face offsetFace(const traits::Face& face, size_t offset) {
  return {face[0] + offset, face[1] + offset, face[2] + offset};
}

traits::Face remapFace(const traits::Face& face,
                       size_t offset,
                       const std::map<size_t, size_t>& remap) {
  return {face[0] >= offset ? remap.at(face[0] - offset) + offset : face[0],
          face[1] >= offset ? remap.at(face[1] - offset) + offset : face[1],
          face[2] >= offset ? remap.at(face[2] - offset) + offset : face[2]};
}

bool faceIsValid(const traits::Face& face) {
  return face[0] != face[1] && face[0] != face[2] && face[1] != face[2];
}

bool allVerticesBelow(const traits::Face& face, size_t threshold) {
  return face[0] < threshold && face[1] < threshold && face[2] < threshold;
}

bool checkFaceAll(const traits::Face& face, const std::function<bool(size_t)>& check) {
  return check(face[0]) && check(face[1]) && check(face[2]);
}

bool checkFaceAny(const traits::Face& face, const std::function<bool(size_t)>& check) {
  return check(face[0]) || check(face[1]) || check(face[2]);
}

traits::Face applyToFace(const traits::Face& face,
                         const std::function<size_t(size_t)>& func) {
  return {func(face[0]), func(face[1]), func(face[2])};
}

}  // namespace kimera_pgmo
