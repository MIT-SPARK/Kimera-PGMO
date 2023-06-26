/**
 * @file   MeshFrontendInterface.cpp
 * @brief  MeshFrontendInterface class: process incoming voxblox meshes and sample it
 * for the mesh parts of the deformation graph
 * @author Yun Chang
 */
#include "kimera_pgmo/MeshFrontendInterface.h"

#include <chrono>
#include <thread>

#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/compression/VoxbloxCompression.h"
#include "kimera_pgmo/compression/VoxelClearingCompression.h"
#include "kimera_pgmo/utils/VoxbloxMsgInterface.h"
#include "kimera_pgmo/utils/VoxbloxMeshInterface.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {

MeshFrontendInterface::MeshFrontendInterface()
    : vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>),
      vertex_stamps_(new std::vector<ros::Time>),
      graph_vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>),
      triangles_(new std::vector<pcl::Vertices>),
      graph_triangles_(new std::vector<pcl::Vertices>),
      vxblx_msg_to_graph_idx_(new VoxbloxIndexMapping),
      vxblx_msg_to_mesh_idx_(new VoxbloxIndexMapping),
      mesh_to_graph_idx_(new IndexMapping),
      init_graph_log_(false),
      init_full_log_(false) {}

bool MeshFrontendInterface::initialize(const MeshFrontendConfig& config) {
  config_ = config;
  switch (config_.graph_compression_method) {
    case 0:
      d_graph_compression_.reset(new OctreeCompression(config_.d_graph_resolution));
      break;
    case 1:
      d_graph_compression_.reset(new VoxbloxCompression(config_.d_graph_resolution));
      break;
    default:
      return false;
  }

  switch (config_.full_compression_method) {
    case 0:
      full_mesh_compression_.reset(new OctreeCompression(config_.mesh_resolution));
      break;
    case 1:
      full_mesh_compression_.reset(new VoxbloxCompression(config_.mesh_resolution));
      break;
    case 2:
      full_mesh_compression_.reset(
          new VoxelClearingCompression(config_.mesh_resolution));
      break;
    default:
      return false;
  }

  // Log header to file
  if (config_.log_output) {
    logGraphProcess();
    logFullProcess();
    init_graph_log_ = true;
    init_full_log_ = true;
  }

  return true;
}

void MeshFrontendInterface::voxbloxCallback(const voxblox_msgs::Mesh& msg) {
  // Start compression threads
  std::thread full_mesh_thread(
      &MeshFrontendInterface::processVoxbloxMeshFull, this, msg);
  std::thread graph_mesh_thread(
      &MeshFrontendInterface::processVoxbloxMeshGraph, this, msg);

  latest_blocks_.clear();
  for (const auto& mesh_block : msg.mesh_blocks) {
    const voxblox::BlockIndex block_index(
        mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);
    latest_blocks_.push_back(block_index);
  }

  full_mesh_thread.join();
  graph_mesh_thread.join();

  if (config_.b_track_mesh_graph_mapping) {
    updateMeshToGraphMappings(latest_blocks_);
  }

  for (const auto& cb_func : output_callbacks_) {
    cb_func(*this, msg.header);
  }
}

// Update full mesh
void MeshFrontendInterface::processVoxbloxMeshFull(const voxblox_msgs::Mesh& msg) {
  // First prune the mesh blocks
  const double msg_time = msg.header.stamp.toSec();
  full_mesh_compression_->pruneStoredMesh(msg_time - config_.time_horizon);

  // Add to full mesh compressor
  auto f_comp_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_triangles =
      std::make_shared<std::vector<pcl::Vertices> >();
  std::shared_ptr<std::vector<size_t> > new_indices =
      std::make_shared<std::vector<size_t> >();
  full_mesh_compression_->compressAndIntegrate(
      msg, new_vertices, new_triangles, new_indices, vxblx_msg_to_mesh_idx_, msg_time);

  auto f_comp_stop = std::chrono::high_resolution_clock::now();
  auto f_comp_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(f_comp_stop - f_comp_start);

  // Update the mesh vertices and surfaces for class variables
  full_mesh_compression_->getVertices(vertices_);
  full_mesh_compression_->getStoredPolygons(triangles_);
  full_mesh_compression_->getVertexStamps(vertex_stamps_);
  assert(vertex_stamps_.size() == vertices_.size());
  // save the active indices
  active_indices_ = full_mesh_compression_->getActiveVerticesIndex();
  invalid_indices_ = full_mesh_compression_->getInvalidIndices();
  if (config_.log_output) {
    logFullProcess(f_comp_duration.count());
  }
}

void MeshFrontendInterface::clearArchivedMeshFull(const voxblox_msgs::Mesh& msg) {
  full_mesh_compression_->clearArchivedBlocks(msg);
}

/*! \brief Publish the factors corresponding to the new edges added to the
 * simplified mesh / deformation graph and also the initial values (positions
 * of the new vertices added to the simplified mesh)
 *  - new_edges: new edges of type Edge (std::pair<Vertex, Vertex>)
 *  - new_indices: new vertices of type Vertex
 *  - graph_vertices: deformation graph vertices
 *  - header: current mesh header
 *  - robot_id: robot for the deformation graph
 *  returns: published pose graph
 */
pose_graph_tools::PoseGraph makePoseGraph(
    const std::vector<Edge>& new_edges,
    const std::vector<size_t>& new_indices,
    const pcl::PointCloud<pcl::PointXYZRGBA>& graph_vertices,
    const std_msgs::Header& header,
    int robot_id) {
  // Create message
  pose_graph_tools::PoseGraph msg;
  msg.header = header;

  // Encode the edges as factors
  msg.edges.resize(new_edges.size());
  for (size_t i = 0; i < new_edges.size(); i++) {
    const Edge& e = new_edges[i];
    pose_graph_tools::PoseGraphEdge pg_edge;
    pg_edge.header = header;

    const size_t& from_node = e.first;
    const size_t& to_node = e.second;

    pg_edge.robot_from = robot_id;
    pg_edge.robot_to = robot_id;
    pg_edge.key_from = from_node;
    pg_edge.key_to = to_node;

    gtsam::Point3 from_node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices.at(from_node));
    gtsam::Point3 to_node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices.at(to_node));
    pg_edge.pose = GtsamToRos(gtsam::Pose3(gtsam::Rot3(), to_node_pos - from_node_pos));

    pg_edge.type = pose_graph_tools::PoseGraphEdge::MESH;

    // Add edge to pose graph
    msg.edges[i] = pg_edge;
  }

  // Encode the new vertices as nodes
  msg.nodes.resize(new_indices.size());
  for (size_t i = 0; i < new_indices.size(); i++) {
    const size_t& n = new_indices[i];
    pose_graph_tools::PoseGraphNode pg_node;
    pg_node.header = header;
    pg_node.robot_id = robot_id;

    pg_node.key = n;

    gtsam::Point3 node_pos = PclToGtsam<pcl::PointXYZRGBA>(graph_vertices.at(n));
    pg_node.pose = GtsamToRos(gtsam::Pose3(gtsam::Rot3(), node_pos));

    // Add node to pose graph
    msg.nodes[i] = pg_node;
  }

  return msg;
}

// Creates and update graph mesh and publish mesh graph
void MeshFrontendInterface::processVoxbloxMeshGraph(const voxblox_msgs::Mesh& msg) {
  // First prune the mesh blocks
  const double msg_time = msg.header.stamp.toSec();
  VoxbloxMsgInterface interface(&msg);
  processMeshGraph(interface, msg_time, msg.header.frame_id);
}

void MeshFrontendInterface::processVoxbloxMeshLayerGraph(
    const voxblox::MeshLayer::Ptr& mesh,
    double msg_time,
    const std::string& frame_id) {
  VoxbloxMeshInterface interface(mesh);
  processMeshGraph(interface, msg_time, frame_id);
}

void MeshFrontendInterface::processMeshGraph(MeshInterface& mesh,
                                             double msg_time,
                                             const std::string& frame_id) {
  d_graph_compression_->pruneStoredMesh(msg_time - config_.time_horizon);

  // Add to deformation graph mesh compressor
  auto g_comp_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_graph_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_graph_triangles =
      std::make_shared<std::vector<pcl::Vertices> >();
  std::shared_ptr<std::vector<size_t> > new_graph_indices =
      std::make_shared<std::vector<size_t> >();

  d_graph_compression_->compressAndIntegrate(mesh,
                                             new_graph_vertices,
                                             new_graph_triangles,
                                             new_graph_indices,
                                             vxblx_msg_to_graph_idx_,
                                             msg_time);

  auto g_comp_stop = std::chrono::high_resolution_clock::now();
  auto g_comp_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(g_comp_stop - g_comp_start);

  // Update the simplified mesh vertices and surfaces for class variables
  d_graph_compression_->getVertices(graph_vertices_);
  d_graph_compression_->getStoredPolygons(graph_triangles_);

  std::vector<Edge> new_graph_edges;
  if (new_graph_indices->size() > 0 && new_graph_triangles->size() > 0) {
    // Add nodes and edges to graph
    new_graph_edges = simplified_mesh_graph_.addPointsAndSurfaces(
        *new_graph_indices.get(), *new_graph_triangles.get());
  }

  if (config_.log_output) {
    logGraphProcess(
        g_comp_duration.count(), new_graph_indices->size(), new_graph_edges.size());
  }

  std_msgs::Header msg_header;
  msg_header.stamp.fromSec(msg_time);
  msg_header.frame_id = frame_id;
  last_mesh_graph_ = makePoseGraph(new_graph_edges,
                                   *new_graph_indices,
                                   *graph_vertices_,
                                   msg_header,
                                   config_.robot_id);
}

void MeshFrontendInterface::updateMeshToGraphMappings(
    const std::vector<BlockIndex>& updated_blocks) {
  for (const auto& block : updated_blocks) {
    for (const auto& remap : vxblx_msg_to_mesh_idx_->at(block)) {
      // TODO(Yun) some mesh vertex might not have graph index if part of the
      // mesh is disconnected and all contained within a block since we remove
      // degenerate faces
      if (vxblx_msg_to_graph_idx_->at(block).count(remap.first)) {
        mesh_to_graph_idx_->insert(
            {remap.second, vxblx_msg_to_graph_idx_->at(block).at(remap.first)});
      }
    }
  }
  return;
}

void MeshFrontendInterface::logGraphProcess(const int& duration,
                                            const size_t& num_indices,
                                            const size_t& num_edges) const {
  std::ofstream file;
  std::string filename = config_.log_path + std::string("/mf_graph_log.csv");
  if (!init_graph_log_) {
    file.open(filename);
    // file format
    file << "num-vertices-simplified,num-vertices-new,num-edges-new,compress-"
            "time(mu-s)\n";
    return;
  }

  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << graph_vertices_->size() << "," << num_indices << "," << num_edges << ","
       << duration << std::endl;
  file.close();
  return;
}

void MeshFrontendInterface::logFullProcess(const int& duration) const {
  std::ofstream file;
  std::string filename = config_.log_path + std::string("/mf_full_log.csv");
  if (!init_full_log_) {
    file.open(filename);
    // file format
    file << "num-vertices-full,num-triangles-full,process-time(mu-s)\n";
    return;
  }

  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << vertices_->size() << "," << triangles_->size() << "," << duration
       << std::endl;
  file.close();
  return;
}

}  // namespace kimera_pgmo
