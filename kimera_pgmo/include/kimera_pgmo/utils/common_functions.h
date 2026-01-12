/**
 * @file   common_functions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <gtsam/inference/Symbol.h>

#include <map>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

const std::map<size_t, char> robot_id_to_prefix = {
    {0, 'a'},
    {1, 'b'},
    {2, 'c'},
    {3, 'd'},
    {4, 'e'},
    {5, 'f'},
    {6, 'g'},
    {7, 'h'},
};

const std::map<char, size_t> robot_prefix_to_id = {
    {'a', 0},
    {'b', 1},
    {'c', 2},
    {'d', 3},
    {'e', 4},
    {'f', 5},
    {'g', 6},
    {'h', 7},
};

const std::map<size_t, char> robot_id_to_vertex_prefix = {
    {0, 's'},
    {1, 't'},
    {2, 'u'},
    {3, 'v'},
    {4, 'w'},
    {5, 'x'},
    {6, 'y'},
    {7, 'z'},
};

const std::map<char, size_t> vertex_prefix_to_id = {
    {'s', 0},
    {'t', 1},
    {'u', 2},
    {'v', 3},
    {'w', 4},
    {'x', 5},
    {'y', 6},
    {'z', 7},
};

/*! \brief Get gtsam prefix for a robot id
 *  - robot_id: id of robot from pose graph messages
 */
inline char GetRobotPrefix(size_t robot_id) {
  if (!robot_id_to_prefix.count(robot_id)) {
    return '\0';
  }
  return robot_id_to_prefix.at(robot_id);
}

/*! \brief Get gtsam prefix (for mesh vertices in deformation graph) for a robot
 * id
 *  - robot_id: id of robot from pose graph messages
 */
inline char GetVertexPrefix(size_t robot_id) {
  if (!robot_id_to_vertex_prefix.count(robot_id)) {
    return '\0';
  }
  return robot_id_to_vertex_prefix.at(robot_id);
}

/*! \brief Check if a GTSAM key represents a mesh vertex
 *  - key: GTSAM key to check
 *  - outputs true if key is a mesh vertex, false otherwise
 */
inline bool IsMeshVertex(gtsam::Key key) {
  return vertex_prefix_to_id.count(gtsam::Symbol(key).chr()) > 0;
}

//! Reindex a face by a static offset
inline traits::Face offsetFace(const traits::Face& face, size_t offset) {
  return {face[0] + offset, face[1] + offset, face[2] + offset};
}

//! Remap all indices above an offset in a face
inline traits::Face remapFace(const traits::Face& face,
                              size_t offset,
                              const std::map<size_t, size_t>& remap) {
  return {face[0] >= offset ? remap.at(face[0] - offset) + offset : face[0],
          face[1] >= offset ? remap.at(face[1] - offset) + offset : face[1],
          face[2] >= offset ? remap.at(face[2] - offset) + offset : face[2]};
}

//! Check if a face is degenerate or not
inline bool faceIsValid(const traits::Face& face) {
  return face[0] != face[1] && face[0] != face[2] && face[1] != face[2];
}

//! Check if a face has indices below a threshold
inline bool allVerticesBelow(const traits::Face& face, size_t threshold) {
  return face[0] < threshold && face[1] < threshold && face[2] < threshold;
}

//! Check that a condition holds for all indices in the face
inline bool checkFaceAll(const traits::Face& face,
                         const std::function<bool(size_t)>& check) {
  return check(face[0]) && check(face[1]) && check(face[2]);
}

//! Check that a condition holds for any index in the face
inline bool checkFaceAny(const traits::Face& face,
                         const std::function<bool(size_t)>& check) {
  return check(face[0]) || check(face[1]) || check(face[2]);
}

//! apply arbitrary function to face
inline traits::Face applyToFace(const traits::Face& face,
                                const std::function<size_t(size_t)>& func) {
  return {func(face[0]), func(face[1]), func(face[2])};
}

}  // namespace kimera_pgmo
