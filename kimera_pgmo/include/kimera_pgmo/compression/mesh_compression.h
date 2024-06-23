/**
 * @file   mesh_compression.h
 * @brief  Simplify and reconstruct meshes
 * @author Yun Chang
 */

#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <spatial_hash/hash.h>

#include <Eigen/Dense>
#include <map>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

#include "kimera_pgmo/utils/common_structs.h"
#include "kimera_pgmo/utils/mesh_interface.h"

namespace kimera_pgmo {

class MeshCompression {
 public:
  using PointCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
  using PointCloudXYZ = pcl::PointCloud<pcl::PointXYZ>;

  MeshCompression(double resolution) : resolution_(resolution) {}

  virtual ~MeshCompression() = default;

  /*! \brief Get the vertices of the compressed full mesh
   *  - vertices: pointer to vertices of full compressed mesh
   */
  inline void getVertices(PointCloud::Ptr vertices) { *vertices = all_vertices_; }

  /*! \brief Get the timestamps of the vertices of the full compressed mesh
   *  - vertex_stamps: pointer to timestamps of the vertices of full compressed
   * mesh
   */
  inline void getTimestamps(std::shared_ptr<std::vector<Timestamp>> vertex_stamps) {
    *vertex_stamps = all_vertex_stamps_;
  }

  /*! \brief Get the vertices currently in the octree (actively being checked
   * for duplication according to resolution)
   *  - vertices: pointer to vertices in octree
   */
  inline void getActiveVertices(PointCloudXYZ::Ptr vertices) {
    *vertices = *active_vertices_xyz_;
  }

  /*! \brief Get the surfaces of the compressed full mesh
   *  - vertices: pointer to surfaces of full compressed mesh
   */
  inline void getStoredPolygons(std::shared_ptr<std::vector<pcl::Vertices>> polygons) {
    *polygons = polygons_;
  }

  /*! \brief Get the timestamps of the active vertices (time of the msg from
   * which the vertices were inserted )
   *  - timestamps: vector of the timestamps indices corresponding to active
   * vertices
   */
  inline void getActiveVerticesTimestamps(
      std::shared_ptr<std::vector<double>> timestamps) {
    *timestamps = active_vertex_stamps_;
  }

  inline size_t getNumVertices() const { return all_vertices_.size(); }

  inline const std::vector<size_t>& getActiveVerticesIndex() const {
    return active_vertices_index_;
  }

  /*! \brief Compress and integrate with the full compressed mesh
   *  - mesh: input mesh as abstract mesh type
   *  - new_vertices: new vertices added after compression
   *  - new_triangles: new mesh surfaces (as triangles) added after compression
   *  - new_indices: indices of the vertices of the compressed partial mesh
   *  - stamp_in_sec: current time stamp in seconds
   */
  virtual void compressAndIntegrate(const MeshInterface& mesh,
                                    PointCloud& new_vertices,
                                    std::vector<pcl::Vertices>& new_triangles,
                                    std::vector<size_t>& new_indices,
                                    HashedIndexMapping& remapping,
                                    double stamp_in_sec);

  /*! \brief Discard parts of the stored compressed full mesh by detection time
   *  - earliest_time_sec: discard all vertices added earlier than this time in
   * seconds
   */
  virtual void pruneStoredMesh(double earliest_time_sec);

  /*! \brief Reinitialize the compression structure (ie. octree or hash cells)
   *  - active_vertices: xyz of the active vertices
   */
  virtual void reInitializeStructure(PointCloudXYZ::Ptr active_vertices) = 0;

  /*! \brief Check if vertex exists in structure
   */
  virtual bool checkIfVertexUnique(const pcl::PointXYZ& v, int* matched_ind) const = 0;

  /*! \brief Updatae structure
   */
  virtual void updateStructure(PointCloudXYZ::Ptr vertices) = 0;

  /*! \brief Check if vertex exists in temporary structure
   */
  virtual bool checkIfVertexTempUnique(const pcl::PointXYZ& v,
                                       int* matched_ind) const = 0;

  /*! \brief Initialize temporary structure
   */
  virtual void initializeTempStructure(PointCloudXYZ::Ptr vertices) = 0;

  /*! \brief Update temporary structure
   */
  virtual void updateTempStructure(PointCloudXYZ::Ptr vertices) = 0;

  /*! \brief Get invalid vertice indices (if any)
   */
  virtual std::vector<size_t> getInvalidIndices() const { return {}; }

  /*! \brief Archive blocks outside active window
   */
  virtual void clearArchivedBlocks(const spatial_hash::BlockIndices& blocks) {}

 protected:
  double resolution_;
  // Vertices in octree (vertices of "active" part of mesh)
  PointCloudXYZ::Ptr active_vertices_xyz_;
  // All verices
  PointCloud all_vertices_;
  // All vertex timestamps
  std::vector<Timestamp> all_vertex_stamps_;
  // Maps index of active vertices to index of all vertices
  std::vector<size_t> active_vertices_index_;
  // Mesh surfaces (all)
  std::vector<pcl::Vertices> polygons_;
  // Keep track of adjacent faces of active part of mesh
  std::map<size_t, std::vector<size_t>> adjacent_polygons_;

  std::vector<double> active_vertex_stamps_;  // timestamps of active vertices
};

using MeshCompressionPtr = std::shared_ptr<MeshCompression>;

}  // namespace kimera_pgmo
