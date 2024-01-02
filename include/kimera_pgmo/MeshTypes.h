/**
 * @file   MeshTypes.h
 * @brief  Convenient typedefs for mesh traits
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstdint>

namespace kimera_pgmo::traits {

using Color = std::array<uint8_t, 3>;
using Face = std::array<size_t, 3>;
using Pos = Eigen::Vector3f;
using Timestamp = uint64_t;
using Label = uint32_t;

}  // namespace kimera_pgmo::traits
