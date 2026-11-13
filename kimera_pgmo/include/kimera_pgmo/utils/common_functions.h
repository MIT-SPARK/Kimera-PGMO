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
char GetRobotPrefix(size_t robot_id);

/*! \brief Get gtsam prefix (for mesh vertices in deformation graph) for a robot
 * id
 *  - robot_id: id of robot from pose graph messages
 */
char GetVertexPrefix(size_t robot_id);

/*! \brief Check if a GTSAM key represents a mesh vertex
 *  - key: GTSAM key to check
 *  - outputs true if key is a mesh vertex, false otherwise
 */
bool IsMeshVertex(gtsam::Key key);

//! Reindex a face by a static offset
traits::Face offsetFace(const traits::Face& face, size_t offset);

//! Remap all indices above an offset in a face
traits::Face remapFace(const traits::Face& face,
                              size_t offset,
                              const std::map<size_t, size_t>& remap);

//! Check if a face is degenerate or not
bool faceIsValid(const traits::Face& face);

//! Check if a face has indices below a threshold
bool allVerticesBelow(const traits::Face& face, size_t threshold);

//! Check that a condition holds for all indices in the face
bool checkFaceAll(const traits::Face& face,
                         const std::function<bool(size_t)>& check);

//! Check that a condition holds for any index in the face
bool checkFaceAny(const traits::Face& face,
                         const std::function<bool(size_t)>& check);

//! apply arbitrary function to face
traits::Face applyToFace(const traits::Face& face,
                         const std::function<size_t(size_t)>& func);

}  // namespace kimera_pgmo
