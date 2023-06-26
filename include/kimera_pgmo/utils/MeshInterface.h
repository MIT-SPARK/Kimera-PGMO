/**
 * @file   VoxbloxMeshInterface.h
 * @brief  Interface wrapper around voxblox mesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <voxblox/core/common.h>
#include <pcl/point_types.h>

namespace kimera_pgmo {

struct MeshInterface {
  virtual const voxblox::BlockIndexList& blockIndices() const = 0;

  virtual void markBlockActive(const voxblox::BlockIndex& block) = 0;

  virtual size_t activeBlockSize() const = 0;

  virtual pcl::PointXYZRGBA getActiveVertex(size_t i) const = 0;
};

}  // namespace kimera_pgmo
