/**
 * @file   VoxbloxCompression.cpp
 * @brief  Combine and simplify meshes based on Voxblox createConnectedMesh
 * @author Yun Chang
 */
#include <iterator>

#include "kimera_pgmo/compression/VoxbloxCompression.h"
#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/VoxbloxUtils.h"

namespace vxb = voxblox;

namespace kimera_pgmo {

VoxbloxCompression::VoxbloxCompression(double resolution)
    : MeshCompression(resolution) {
  active_vertices_xyz_.reset(new PointCloudXYZ);
}

VoxbloxCompression::~VoxbloxCompression() {}

void VoxbloxCompression::reInitializeStructure(
    PointCloudXYZ::Ptr active_vertices) {
  // Reset cell hash
  cell_hash_.clear();
  size_t idx = 0;
  for (const auto& p : active_vertices->points) {
    const vxb::LongIndex& vertex_3D_index =
        PclPtToVoxbloxLongIndex<pcl::PointXYZ>(p, resolution_);
    cell_hash_.emplace(vertex_3D_index, idx);
    idx++;
  }
}

bool VoxbloxCompression::checkIfVertexUnique(const pcl::PointXYZ& v,
                                             int* matched_ind) const {
  const vxb::LongIndex& vertex_3D_index =
      PclPtToVoxbloxLongIndex<pcl::PointXYZ>(v, resolution_);
  vxb::LongIndexHashMapType<size_t>::type::const_iterator it =
      cell_hash_.find(vertex_3D_index);
  if (it == cell_hash_.end()) {
    return true;
  } else {
    *matched_ind = it->second;
    return false;
  }
}

void VoxbloxCompression::updateStructure(PointCloudXYZ::Ptr vertices) {
  size_t vertex_index = vertices->size() - 1;
  vxb::LongIndex v_3d_index = PclPtToVoxbloxLongIndex<pcl::PointXYZ>(
      vertices->points[vertex_index], resolution_);
  cell_hash_.emplace(v_3d_index, vertex_index);
}

/*! \brief Check if vertex exists in temporary structure
 */
bool VoxbloxCompression::checkIfVertexTempUnique(const pcl::PointXYZ& v,
                                                 int* matched_ind) const {
  const vxb::LongIndex& vertex_3D_index =
      PclPtToVoxbloxLongIndex<pcl::PointXYZ>(v, resolution_);
  vxb::LongIndexHashMapType<size_t>::type::const_iterator it =
      temp_cell_hash_.find(vertex_3D_index);
  if (it == temp_cell_hash_.end()) {
    return true;
  } else {
    *matched_ind = it->second;
    return false;
  }
}

void VoxbloxCompression::initializeTempStructure(PointCloudXYZ::Ptr vertices) {
  temp_cell_hash_.clear();
  size_t idx = 0;
  for (const auto& p : vertices->points) {
    const vxb::LongIndex& vertex_3D_index =
        PclPtToVoxbloxLongIndex<pcl::PointXYZ>(p, resolution_);
    temp_cell_hash_.emplace(vertex_3D_index, idx);
    idx++;
  }
}

void VoxbloxCompression::updateTempStructure(PointCloudXYZ::Ptr vertices) {
  size_t vertex_index = vertices->size() - 1;
  vxb::LongIndex v_3d_index = PclPtToVoxbloxLongIndex<pcl::PointXYZ>(
      vertices->points[vertex_index], resolution_);
  temp_cell_hash_.emplace(v_3d_index, vertex_index);
}

}  // namespace kimera_pgmo