/**
 * @file   common_functions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

char GetRobotPrefix(size_t robot_id) {
  if (!robot_id_to_prefix.count(robot_id)) {
    return '\0';
  }

  return robot_id_to_prefix.at(robot_id);
}

char GetVertexPrefix(size_t robot_id) {
  if (!robot_id_to_vertex_prefix.count(robot_id)) {
    return '\0';
  }

  return robot_id_to_vertex_prefix.at(robot_id);
}

bool IsMeshVertex(gtsam::Key key) {
  return vertex_prefix_to_id.count(gtsam::Symbol(key).chr()) > 0;
}

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
