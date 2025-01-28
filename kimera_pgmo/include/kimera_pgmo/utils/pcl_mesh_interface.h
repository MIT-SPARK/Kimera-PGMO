/**
 * @file   pcl_mesh_interface.h
 * @brief  Interface wrapper around pcl mesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "kimera_pgmo/utils/mesh_interface.h"

namespace kimera_pgmo {

class PclMeshInterface : public MeshInterface {
 public:
  PclMeshInterface(const pcl::PolygonMesh& mesh)
      : cloud_(new pcl::PointCloud<pcl::PointXYZRGBA>()), faces_(mesh.polygons) {
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud_);
    indices_.push_back(BlockIndex::Zero());
  }

  const BlockIndices& blockIndices() const override { return indices_; }

  void markBlockActive(const BlockIndex& block) const override {}

  size_t activeBlockSize() const override { return 3 * faces_.size(); }

  pcl::PointXYZRGBA getActiveVertex(size_t i) const override {
    const auto face_idx = i / 3;
    const auto v_idx = i % 3;
    const auto idx = faces_.at(face_idx).vertices.at(v_idx);
    return cloud_->at(idx);
  }

  MeshInterface::Ptr clone() const override {
    return std::make_shared<PclMeshInterface>(*this);
  }

 protected:
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_;
  std::vector<pcl::Vertices> faces_;
  BlockIndices indices_;
};

}  // namespace kimera_pgmo
