/**
 * @file   MeshDelta.h
 * @brief  Struct and functions for updating mesh
 * @author Nathan Hughes
 */

#pragma once

#include <pcl/Vertices.h>
#include <pcl/pcl_base.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>

#include <optional>
#include <vector>

#include "kimera_pgmo/KimeraPgmoMeshDelta.h"
#include "kimera_pgmo/utils/CommonStructs.h"

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

  MeshDelta(const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
            const std::vector<Timestamp>& stamps,
            const std::vector<pcl::Vertices>& faces,
            std::optional<std::vector<uint32_t>> semantics = std::nullopt);

  void updateVertices(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                      std::vector<Timestamp>* stamps = nullptr,
                      std::vector<uint32_t>* semantics = nullptr) const;

  void updateMesh(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                  std::vector<Timestamp>& stamps,
                  std::vector<pcl::Vertices>& faces,
                  std::vector<uint32_t>* semantics = nullptr) const;

  size_t addVertex(Timestamp timestamp_ns,
                   const pcl::PointXYZRGBA& point,
                   std::optional<uint32_t> semantics = std::nullopt,
                   bool archive = false);

  void addFace(const Face& face, bool archive = false);

  bool hasSemantics() const;

  size_t getNumArchivedVertices() const;

  size_t getNumArchivedFaces() const;

  size_t getTotalArchivedVertices() const;

  size_t getTotalArchivedFaces() const;

  pcl::IndicesPtr getActiveIndices() const;

  void checkFaces(const std::string& name) const;

  void validate(pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
                std::vector<Timestamp>& stamps,
                std::vector<pcl::Vertices>& faces,
                std::vector<uint32_t>* semantics = nullptr) const;

  KimeraPgmoMeshDelta toRosMsg(Timestamp timestamp_ns) const;

  size_t vertex_start = 0;
  size_t face_start = 0;

  std::vector<pcl::PointXYZRGBA> vertex_updates;
  std::vector<Timestamp> stamp_updates;
  std::vector<uint32_t> semantic_updates;
  std::vector<Face> face_updates;
  std::vector<Face> face_archive_updates;
  std::map<size_t, size_t> prev_to_curr;
  std::set<size_t> deleted_indices;
  std::set<size_t> observed_indices;
  std::set<size_t> new_indices;

 protected:
  size_t num_archived_vertices_ = 0;
};

std::ostream& operator<<(std::ostream& out, const MeshDelta& delta);

}  // namespace kimera_pgmo
