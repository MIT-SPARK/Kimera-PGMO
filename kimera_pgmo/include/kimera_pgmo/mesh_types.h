/**
 * @file   mesh_types.h
 * @brief  Convenient typedefs for mesh traits
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <array>
#include <cstdint>
#include <unordered_map>

#include <Eigen/Dense>

namespace kimera_pgmo {
namespace traits {

using Color = std::array<uint8_t, 4>;  // RGBA
using Face = std::array<size_t, 3>;
using Pos = Eigen::Vector3f;
using Timestamp = uint64_t;
using Label = uint32_t;

/**
 * @brief Struct describing which vertex trait fields are valid
 */
struct VertexProperties {
  bool has_color = false;
  bool has_stamp = false;
  bool has_label = false;
  bool has_first_seen_stamp = false;

  VertexProperties& operator|=(const VertexProperties& other);
  VertexProperties& operator&=(const VertexProperties& other);
};

bool operator==(const VertexProperties& lhs, const VertexProperties& rhs);

/**
 * @brief Struct to bundle all optional vertex traits to supplement the mandatory
 * position. The mesh itself determines which of these are populated.
 * TODO(lschmid): Note that the mesh IO is hard coded to only save these traits
 * currently.
 * TODO(lschmid): Double check that internally traits are passed correctly so they could
 * in the future be truly polymorphic.
 */
struct VertexTraits {
  VertexProperties properties;
  Color color = {0, 0, 0, 0};
  Timestamp stamp = 0;
  Label label = 0;
  Timestamp first_seen_stamp = std::numeric_limits<Timestamp>::max();
};

bool operator==(const VertexTraits& lhs, const VertexTraits& rhs);

struct Vertex {
  traits::Pos pos = traits::Pos::Zero();
  traits::VertexTraits traits;
};

}  // namespace traits

using IndexMapping = std::unordered_map<size_t, size_t>;

}  // namespace kimera_pgmo
