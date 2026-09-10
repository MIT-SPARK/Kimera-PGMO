/**
 * @file   mesh_interface.h
 * @brief  Interface wrapper around meshes
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once

#include <pcl/point_types.h>

#include <array>
#include <optional>
#include <stdexcept>

#include "kimera_pgmo/hashing.h"

namespace kimera_pgmo {

struct MeshInterface {
  using Ptr = std::shared_ptr<MeshInterface>;

  virtual ~MeshInterface() = default;

  virtual const BlockIndices& blockIndices() const = 0;

  virtual void markBlockActive(const BlockIndex& block) const = 0;

  virtual size_t activeBlockSize() const = 0;

  virtual pcl::PointXYZRGBA getActiveVertex(size_t i) const = 0;

  virtual bool hasSemantics() const { return false; }

  virtual std::optional<uint32_t> getActiveSemantics(size_t /* index */) const {
    return std::nullopt;
  }

  virtual Ptr clone() const = 0;

  // By default, each consecutive group of three vertices defines a face.
  // Indexed meshes override both methods to expose their explicit faces.
  virtual size_t activeBlockNumFaces() const {
    const auto size = activeBlockSize();
    if (size % 3 != 0) {
      throw std::invalid_argument("Mesh vertex count does not define complete faces");
    }
    return size / 3;
  }

  virtual std::array<size_t, 3> getActiveFace(size_t i) const {
    const auto index = 3 * i;
    return {index, index + 1, index + 2};
  }
};

}  // namespace kimera_pgmo
