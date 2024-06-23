/**
 * @file   mesh_frontend.h
 * @brief  MeshFrontend class: process incoming meshes
 * @author Yun Chang
 */
#pragma once

#include <config_utilities/types/enum.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pose_graph_tools/pose_graph.h>
#include <spatial_hash/types.h>

#include "kimera_pgmo/compression/mesh_compression.h"
#include "kimera_pgmo/hashing.h"
#include "kimera_pgmo/utils/common_structs.h"

namespace kimera_pgmo {

class MeshFrontendInterface {
  friend class MeshFrontendTest;

 public:
  enum class FullCompressionMethod { OCTREE, BLOCK, VOXEL_CLEARING };
  enum class GraphCompressionMethod { OCTREE, BLOCK };
  struct Config {
    int robot_id = 0;
    //! only merge meshes for the blocks detected within defined time horizon (secs)
    double time_horizon = 10.0;
    bool track_mesh_graph_mapping = true;
    std::string log_path = "";
    bool log_output = true;
    FullCompressionMethod full_compression_method = FullCompressionMethod::BLOCK;
    GraphCompressionMethod graph_compression_method = GraphCompressionMethod::BLOCK;
    double d_graph_resolution = 3.0;
    double mesh_resolution = 0.2;
  } const config;

  explicit MeshFrontendInterface(const Config& config);

  virtual ~MeshFrontendInterface() = default;

  /*! \brief Main callback of this class: receives the updated incremental mesh
   *  - mesh: Latest mesh
   */
  void update(const MeshInterface& mesh, double time_in_sec);

  /*! \brief Get last mesh graph created in callback
   */
  inline pose_graph_tools::PoseGraph::Ptr getLastProcessedMeshGraph() const {
    return last_mesh_graph_;
  }

  /*! \brief Get the mappings from vxblx msg to graph index for tracking.
   */
  inline const HashedIndexMapping& getVoxbloxMsgToGraphMapping() const {
    return *msg_to_graph_idx_;
  }

 protected:
  /*! \brief Process the latest incremental mesh from the
   * callback and add the partial mesh to the graph mesh and compress
   *  - msg: Arbitrary mesh interface
   *  - time_in_sec: message time
   */
  void updateGraph(const MeshInterface::Ptr mesh, double time_in_sec);

  /*! \brief Process the latest incremental mesh from the
   * callback and add the partial mesh to the full mesh and compress
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   */
  void updateFullMesh(const MeshInterface::Ptr mesh, double time_in_sec);

  /*! \brief Update full mesh to mesh graph index mappings
   */
  void updateMeshToGraphMappings(const BlockIndices& updated_blocks);

  /*! \brief Log the stats and the timing for graph compression thread
   *  - duration: callback time (mu-s)
   *  - num_indices: number of new indices to add to deformation graph
   *  - num_edges: number of new edges to add to deformation graph
   */
  void logGraphProcess(int duration = 0,
                       size_t num_indices = 0,
                       size_t num_edges = 0) const;

  /*! \brief Log the stats and timing for full mesh processing thread
   *  - duration: callback time (mu-s)
   */
  void logFullProcess(int duration = 0) const;

  MeshCompressionPtr full_mesh_compression_;  // Allow compression of full mesh

  MeshCompressionPtr d_graph_compression_;  // Compression to get simplified
                                            // mesh for deformation graph

  Graph simplified_mesh_graph_;  // Graph of simplified mesh (edges are the
                                 // factors in deformation graph)

  // Vertices of full mesh
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices_;
  // Triangles (connections) of full mesh
  std::shared_ptr<std::vector<pcl::Vertices>> triangles_;
  // Vertices time stamps of full mesh
  std::shared_ptr<std::vector<Timestamp>> vertex_stamps_;

  // Vertices of simplified mesh used for the deformation graph
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr graph_vertices_;
  // Triangles of the simplified mesh used for the deformation graph
  std::shared_ptr<std::vector<pcl::Vertices>> graph_triangles_;
  // Vertices time stamps of deformation graph
  std::shared_ptr<std::vector<Timestamp>> graph_stamps_;

  // Last pose graph msg created for testing purposes
  pose_graph_tools::PoseGraph::Ptr last_mesh_graph_;

  // Book keeping for indices
  std::shared_ptr<HashedIndexMapping> msg_to_graph_idx_;
  std::shared_ptr<HashedIndexMapping> msg_to_mesh_idx_;
  std::shared_ptr<IndexMapping> mesh_to_graph_idx_;
  BlockIndices latest_blocks_;

  bool init_graph_log_;
  bool init_full_log_;

  std::vector<size_t> active_indices_;
  std::vector<size_t> invalid_indices_;

 private:
  inline static const auto enum_init_ =
      config::Enum<FullCompressionMethod>::Initializer(
          {{FullCompressionMethod::OCTREE, "OCTREE"},
           {FullCompressionMethod::BLOCK, "BLOCK"},
           {FullCompressionMethod::VOXEL_CLEARING, "VOXEL_CLEARING"}});
  inline static const auto enum_init_2 =
      config::Enum<GraphCompressionMethod>::Initializer(
          {{GraphCompressionMethod::OCTREE, "OCTREE"},
           {GraphCompressionMethod::BLOCK, "BLOCK"}});
};

void declare_config(MeshFrontendInterface::Config& config);

}  // namespace kimera_pgmo
