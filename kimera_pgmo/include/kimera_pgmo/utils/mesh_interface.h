/**
 * @file   voxblox_mesh_interface.h
 * @brief  Interface wrapper around voxblox mesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <pcl/point_types.h>

#include <optional>

namespace kimera_pgmo {

using BlockIndex = Eigen::Vector3i;
using BlockIndices =
    std::vector<Eigen::Vector3i, Eigen::aligned_allocator<Eigen::Vector3i>>;

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
};

}  // namespace kimera_pgmo
