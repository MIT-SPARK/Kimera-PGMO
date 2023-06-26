/**
 * @file   MeshFrontend.h
 * @brief  MeshFrontend class: process incoming voxblox meshes
 * @author Yun Chang
 */
#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/ros.h>
#include <voxblox/core/block_hash.h>
#include <voxblox/mesh/mesh_layer.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/compression/MeshCompression.h"
#include "kimera_pgmo/utils/CommonStructs.h"

namespace kimera_pgmo {

using voxblox::BlockIndex;

struct MeshFrontendConfig {
  int robot_id = 0;
  double time_horizon = 10.0;  // only merge meshes for the blocks detected
                               // within defined time horizon (secs)
  bool b_track_mesh_graph_mapping = true;
  std::string log_path;
  bool log_output;
  int full_compression_method = 2;  // 0 for octree, 1 for voxblox, 2 for voxel clearing
  int graph_compression_method = 1;  // 0 for octree, 1 for voxblox
  double d_graph_resolution = 3.0;
  double mesh_resolution = 0.2;
};

class MeshFrontendInterface {
  friend class MeshFrontendTest;
  friend class MeshFrontendPublisher;
  using OutputCallback =
      std::function<void(const MeshFrontendInterface&, const std_msgs::Header&)>;

 public:
  MeshFrontendInterface();

  virtual ~MeshFrontendInterface() = default;

  bool initialize(const MeshFrontendConfig& config);

  /*! \brief Main callback of this class: receives the updated incremental mesh
   * from Voxblox or Kimera-Semantics
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   */
  void voxbloxCallback(const voxblox_msgs::Mesh& msg);

  /*! \brief Let mesh compression clear archived blocks
   *
   *  \warning Not threadsafe, use with caution
   */
  void clearArchivedMeshFull(const voxblox_msgs::Mesh& msg);

  inline void addOutputCallback(const OutputCallback& callback) {
    output_callbacks_.push_back(callback);
  }

  /*! /brief Get curent frontend vertices
   *  /returns Current vertex pointcloud
   */
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getFullMeshVertices() const {
    return vertices_;
  }

  inline const std::vector<pcl::Vertices>& getFullMeshFaces() const {
    return *triangles_;
  }

  inline const std::vector<ros::Time>& getFullMeshTimes() const {
    return *vertex_stamps_;
  }

  /*! /brief Get active indices
   *  /returns Active indices (i.e. inside frontend time-horizon) of the full
   * mesh
   */
  inline const std::vector<size_t>& getActiveFullMeshVertices() const {
    return active_indices_;
  }

  /*! /brief Get invalid indices in the mesh
   *  /returns Indices of the mesh that mesh compression has deleted
   */
  inline const std::vector<size_t>& getInvalidIndices() const {
    return invalid_indices_;
  }

  /*! \brief Get the time horizion (in seconds) of the mesh compression
   */
  inline double getMeshTimeHorizon() const { return config_.time_horizon; }

  /*! \brief Get the mappings from vxblx msg to graph index for tracking.
   */
  inline const VoxbloxIndexMapping& getVoxbloxMsgToGraphMapping() const {
    return *vxblx_msg_to_graph_idx_;
  }

  /*! \brief Get the mappings from vxblx msg to graph index for tracking.
   */
  inline const VoxbloxIndexMapping& getVoxbloxMsgToMeshMapping() const {
    return *vxblx_msg_to_mesh_idx_;
  }

  /*! \brief Get the mappings from vxblx msg to graph index for tracking.
   */
  inline const IndexMapping& getFullMeshToGraphMapping() const {
    return *mesh_to_graph_idx_;
  }

  /*! \brief Get last mesh graph created in voxblox callback
   */
  inline pose_graph_tools::PoseGraph getLastProcessedMeshGraph() const {
    return last_mesh_graph_;
  }

  /*! \brief Process the latest incremental mesh from the
   * callback and add the partial mesh to the graph mesh and compress
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   */
  void processVoxbloxMeshGraph(const voxblox_msgs::Mesh& msg);

  /*! \brief Process the latest incremental mesh from the
   * callback and add the partial mesh to the graph mesh and compress
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   *  - time_in_sec: message time
   *  - frame_id: message frame id
   */
  void processVoxbloxMeshLayerGraph(const voxblox::MeshLayer::Ptr& mesh,
                                    double time_in_sec,
                                    const std::string& frame_id = "world");

 protected:
  void processMeshGraph(MeshInterface& mesh,
                        double time_in_sec,
                        const std::string& frame_id);

  /*! \brief Process the latest incremental mesh from the
   * callback and add the partial mesh to the full mesh and compress
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   */
  void processVoxbloxMeshFull(const voxblox_msgs::Mesh& msg);

  /*! \brief Update full mesh to mesh graph index mappings
   */
  void updateMeshToGraphMappings(const std::vector<BlockIndex>& updated_blocks);

  /*! \brief Log the stats and the timing for graph compression thread
   *  - duration: callback time (mu-s)
   *  - num_indices: number of new indices to add to deformation graph
   *  - num_edges: number of new edges to add to deformation graph
   */
  void logGraphProcess(const int& duration = 0,
                       const size_t& num_indices = 0,
                       const size_t& num_edges = 0) const;

  /*! \brief Log the stats and timing for full mesh processing thread
   *  - duration: callback time (mu-s)
   */
  void logFullProcess(const int& duration = 0) const;

  MeshFrontendConfig config_;

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
  std::shared_ptr<std::vector<ros::Time>> vertex_stamps_;
  // Vertices of simplified mesh used for the deformation graph
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr graph_vertices_;
  // Triangles of the simplified mesh used for the deformation graph
  std::shared_ptr<std::vector<pcl::Vertices>> graph_triangles_;

  // Last pose graph msg created for testing purposes
  pose_graph_tools::PoseGraph last_mesh_graph_;

  // Book keeping for indices
  std::shared_ptr<VoxbloxIndexMapping> vxblx_msg_to_graph_idx_;
  std::shared_ptr<VoxbloxIndexMapping> vxblx_msg_to_mesh_idx_;
  std::shared_ptr<IndexMapping> mesh_to_graph_idx_;
  std::vector<BlockIndex> latest_blocks_;

  bool init_graph_log_;
  bool init_full_log_;

  std::vector<size_t> active_indices_;
  std::vector<size_t> invalid_indices_;
  std::vector<OutputCallback> output_callbacks_;
};

}  // namespace kimera_pgmo
