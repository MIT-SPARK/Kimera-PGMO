/**
 * @file   mesh_types.h
 * @brief  Convenient typedefs for mesh traits
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <unordered_map>

namespace kimera_pgmo {
namespace traits {

using Color = std::array<uint8_t, 4>;  // RGBA
using Face = std::array<size_t, 3>;
using Pos = Eigen::Vector3f;
using Timestamp = uint64_t;
using Label = uint32_t;

/**
 * @brief Struct to bundle all optional vertex traits to supplement the mandatory
 * position. Custom traits can derive from this struct and will be passed correctly
 * through pgmo.
 * TODO(lschmid): Note that the mesh IO is hard coded to only save these traits
 * currently.
 * TODO(lschmid): Double check that internally traits are passed correctly so they could
 * in the future be truly polymorphic.
 */
struct VertexTraits {
  VertexTraits() = default;
  virtual ~VertexTraits() = default;

  std::optional<Color> color;
  std::optional<Timestamp> stamp;
  std::optional<Label> label;
};

}  // namespace traits

using IndexMapping = std::unordered_map<size_t, size_t>;

}  // namespace kimera_pgmo
