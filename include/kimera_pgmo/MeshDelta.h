/**
 * @file   MeshDelta.h
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#pragma once

#include "kimera_pgmo/KimeraPgmoMeshDelta.h"

#include <vector>

#include <ros/ros.h>

#include <pcl/Vertices.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kimera_pgmo {

struct Face {
  Face(size_t v1, size_t v2, size_t v3);

  Face(const std::vector<size_t>& indices, size_t i);

  bool valid() const;

  void fill(std::vector<uint32_t>& other) const;

  uint32_t v1;
  uint32_t v2;
  uint32_t v3;
};

std::ostream& operator<<(std::ostream& out, const Face& face);

class MeshDelta {
 public:
  using Ptr = std::shared_ptr<MeshDelta>;

  MeshDelta() = default;

  MeshDelta(size_t vertex_start, size_t face_start);

  MeshDelta(const KimeraPgmoMeshDelta& msg);

  void updateVertices(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                      std::vector<ros::Time>* stamps = nullptr) const;

  void updateMesh(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                  std::vector<ros::Time>& stamps,
                  std::vector<pcl::Vertices>& faces) const;

  size_t addVertex(uint64_t timestamp_ns,
                   const pcl::PointXYZRGBA& point,
                   bool archive = false);

  void addFace(const Face& face, bool archive = false);

  size_t getNumArchivedVertices() const;

  size_t getNumArchivedFaces() const;

  size_t getTotalArchivedVertices() const;

  size_t getTotalArchivedFaces() const;

  pcl::IndicesPtr getActiveIndices() const;

  void checkFaces(const std::string& name) const;

  void validate(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                std::vector<ros::Time>& stamps,
                std::vector<pcl::Vertices>& faces) const;

  KimeraPgmoMeshDelta toRosMsg(uint64_t timestamp_ns) const;

  size_t vertex_start = 0;
  size_t face_start = 0;

  std::vector<pcl::PointXYZRGBA> vertex_updates;
  std::vector<uint64_t> stamp_updates;
  std::vector<Face> face_updates;
  std::vector<Face> face_archive_updates;
  std::map<size_t, size_t> prev_to_curr;
  std::set<size_t> deleted_indices;

 protected:
  size_t num_archived_vertices_ = 0;
};

}  // namespace kimera_pgmo
