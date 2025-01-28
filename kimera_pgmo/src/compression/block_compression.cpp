/**
 * @file   block_compression.cpp
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 */
#include "kimera_pgmo/compression/block_compression.h"

#include <iterator>

#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

BlockCompression::BlockCompression(double resolution)
    : MeshCompression(resolution), grid_(resolution) {
  active_vertices_xyz_.reset(new PointCloudXYZ);
}

LongIndex BlockCompression::toIndex(const pcl::PointXYZ& p) const {
  return grid_.toIndex(spatial_hash::Point(p.x, p.y, p.z));
}

void BlockCompression::reInitializeStructure(PointCloudXYZ::Ptr active_vertices) {
  // Reset cell hash
  cell_hash_.clear();
  size_t idx = 0;
  for (const auto& p : active_vertices->points) {
    cell_hash_.emplace(toIndex(p), idx);
    idx++;
  }
}

bool BlockCompression::checkIfVertexUnique(const pcl::PointXYZ& v,
                                           int* matched_ind) const {
  const LongIndex vertex_3D_index = toIndex(v);
  const auto it = cell_hash_.find(vertex_3D_index);
  if (it == cell_hash_.end()) {
    return true;
  } else {
    *matched_ind = it->second;
    return false;
  }
}

void BlockCompression::updateStructure(PointCloudXYZ::Ptr vertices) {
  const size_t vertex_index = vertices->size() - 1;
  const LongIndex v_3d_index = toIndex(vertices->points[vertex_index]);
  cell_hash_.emplace(v_3d_index, vertex_index);
}

/*! \brief Check if vertex exists in temporary structure
 */
bool BlockCompression::checkIfVertexTempUnique(const pcl::PointXYZ& v,
                                               int* matched_ind) const {
  const LongIndex vertex_3D_index = toIndex(v);
  const auto it = temp_cell_hash_.find(vertex_3D_index);
  if (it == temp_cell_hash_.end()) {
    return true;
  } else {
    *matched_ind = it->second;
    return false;
  }
}

void BlockCompression::initializeTempStructure(PointCloudXYZ::Ptr vertices) {
  temp_cell_hash_.clear();
  size_t idx = 0;
  for (const auto& p : vertices->points) {
    const LongIndex vertex_3D_index = toIndex(p);
    temp_cell_hash_.emplace(vertex_3D_index, idx);
    idx++;
  }
}

void BlockCompression::updateTempStructure(PointCloudXYZ::Ptr vertices) {
  size_t vertex_index = vertices->size() - 1;
  const LongIndex v_3d_index = toIndex(vertices->points[vertex_index]);
  temp_cell_hash_.emplace(v_3d_index, vertex_index);
}

}  // namespace kimera_pgmo
