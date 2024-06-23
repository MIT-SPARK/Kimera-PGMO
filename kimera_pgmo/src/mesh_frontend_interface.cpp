/**
 * @file   mesh_frontend_interface.cpp
 * @brief  MeshFrontendInterface class: process incoming meshes and sample it
 * for the mesh parts of the deformation graph
 * @author Yun Chang
 */
#include "kimera_pgmo/mesh_frontend_interface.h"

#include <config_utilities/config.h>
#include <config_utilities/validation.h>

#include <chrono>
#include <fstream>
#include <thread>

#include "kimera_pgmo/compression/block_compression.h"
#include "kimera_pgmo/compression/octree_compression.h"
#include "kimera_pgmo/compression/voxel_clearing_compression.h"
#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo {

using FullCompressionMethod = MeshFrontendInterface::FullCompressionMethod;
using GraphCompressionMethod = MeshFrontendInterface::GraphCompressionMethod;

void declare_config(MeshFrontendInterface::Config& config) {
  using namespace config;
  name("MeshFrontendInterface::Config");
  field(config.time_horizon, "time_horizon");
  field(config.track_mesh_graph_mapping, "track_mesh_graph_mapping");
  field(config.log_path, "log_path");
  field(config.log_output, "log_output");
  field<Enum<FullCompressionMethod>>(config.full_compression_method,
                                     "full_compression_method");
  field<Enum<GraphCompressionMethod>>(config.graph_compression_method,
                                      "graph_compression_method");
  field(config.d_graph_resolution, "d_graph_resolution");
  field(config.mesh_resolution, "mesh_resolution");
  // validation
  check(config.time_horizon, GT, 0.0, "time_horizon");
  check(config.mesh_resolution, GT, 0.0, "mesh_resolution");
}

MeshCompressionPtr createFullCompression(FullCompressionMethod method,
                                         double resolution) {
  switch (method) {
    case FullCompressionMethod::OCTREE:
      return std::make_shared<OctreeCompression>(resolution);
    case FullCompressionMethod::BLOCK:
      return std::make_shared<BlockCompression>(resolution);
    case FullCompressionMethod::VOXEL_CLEARING:
      return std::make_shared<VoxelClearingCompression>(resolution);
    default:
      return nullptr;
  }
}

MeshCompressionPtr createGraphCompression(GraphCompressionMethod method,
                                          double resolution) {
  switch (method) {
    case GraphCompressionMethod::OCTREE:
      return std::make_shared<OctreeCompression>(resolution);
    case GraphCompressionMethod::BLOCK:
      return std::make_shared<BlockCompression>(resolution);
    default:
      return nullptr;
  }
}

MeshFrontendInterface::MeshFrontendInterface(const Config& config)
    : config(config::checkValid(config)),
      vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>()),
      triangles_(new std::vector<pcl::Vertices>()),
      vertex_stamps_(new std::vector<Timestamp>()),
      graph_vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>()),
      graph_triangles_(new std::vector<pcl::Vertices>()),
      graph_stamps_(new std::vector<Timestamp>()),
      msg_to_graph_idx_(new HashedIndexMapping()),
      msg_to_mesh_idx_(new HashedIndexMapping()),
      mesh_to_graph_idx_(new IndexMapping()),
      init_graph_log_(false),
      init_full_log_(false) {
  d_graph_compression_ = createGraphCompression(config.graph_compression_method,
                                                config.d_graph_resolution);
  assert(d_graph_compression_ != nullptr);

  full_mesh_compression_ =
      createFullCompression(config.full_compression_method, config.mesh_resolution);
  assert(full_mesh_compression_ != nullptr);

  // Log header to file
  if (config.log_output) {
    logGraphProcess();
    logFullProcess();
    init_graph_log_ = true;
    init_full_log_ = true;
  }
}

void MeshFrontendInterface::update(const MeshInterface& mesh, double time_s) {
  // Start compression threads
  std::thread full_thread(
      &MeshFrontendInterface::updateFullMesh, this, mesh.clone(), time_s);
  std::thread graph_thread(
      &MeshFrontendInterface::updateGraph, this, mesh.clone(), time_s);

  latest_blocks_ = mesh.blockIndices();
  
  full_thread.join();
  graph_thread.join();

  if (config.track_mesh_graph_mapping) {
    updateMeshToGraphMappings(latest_blocks_);
  }
}

// Update full mesh
void MeshFrontendInterface::updateFullMesh(const MeshInterface::Ptr mesh,
                                           double time_in_sec) {
  if (!mesh) {
    return;
  }

  // First prune the mesh blocks
  full_mesh_compression_->pruneStoredMesh(time_in_sec - config.time_horizon);

  std::vector<size_t> new_indices;
  std::vector<pcl::Vertices> new_triangles;
  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;

  // Add to full mesh compressor
  auto f_comp_start = std::chrono::high_resolution_clock::now();
  full_mesh_compression_->compressAndIntegrate(
      *mesh, new_vertices, new_triangles, new_indices, *msg_to_mesh_idx_, time_in_sec);
  auto f_comp_stop = std::chrono::high_resolution_clock::now();

  auto f_comp_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(f_comp_stop - f_comp_start);

  // Update the mesh vertices and surfaces for class variables
  full_mesh_compression_->getVertices(vertices_);
  full_mesh_compression_->getStoredPolygons(triangles_);
  full_mesh_compression_->getTimestamps(vertex_stamps_);
  assert(vertex_stamps_->size() == vertices_->size());
  // save the active indices
  active_indices_ = full_mesh_compression_->getActiveVerticesIndex();
  invalid_indices_ = full_mesh_compression_->getInvalidIndices();
  if (config.log_output) {
    logFullProcess(f_comp_duration.count());
  }
}

void MeshFrontendInterface::updateGraph(const MeshInterface::Ptr mesh,
                                        double msg_time) {
  if (!mesh) {
    return;
  }

  d_graph_compression_->pruneStoredMesh(msg_time - config.time_horizon);

  // Add to deformation graph mesh compressor
  std::vector<size_t> new_indices;
  std::vector<pcl::Vertices> new_triangles;
  pcl::PointCloud<pcl::PointXYZRGBA> new_graph_vertices;

  auto g_comp_start = std::chrono::high_resolution_clock::now();
  d_graph_compression_->compressAndIntegrate(*mesh,
                                             new_graph_vertices,
                                             new_triangles,
                                             new_indices,
                                             *msg_to_graph_idx_,
                                             msg_time);
  auto g_comp_stop = std::chrono::high_resolution_clock::now();

  auto g_comp_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(g_comp_stop - g_comp_start);

  // Update the simplified mesh vertices and surfaces for class variables
  d_graph_compression_->getVertices(graph_vertices_);
  d_graph_compression_->getStoredPolygons(graph_triangles_);
  d_graph_compression_->getTimestamps(graph_stamps_);

  std::vector<Edge> new_edges;
  if (new_indices.size() > 0 && new_triangles.size() > 0) {
    // Add nodes and edges to graph
    new_edges = simplified_mesh_graph_.addPointsAndSurfaces(new_indices, new_triangles);
  }

  if (config.log_output) {
    logGraphProcess(g_comp_duration.count(), new_indices.size(), new_edges.size());
  }

  last_mesh_graph_ = makePoseGraph(
      config.robot_id, msg_time, new_edges, new_indices, *graph_vertices_);
}

void MeshFrontendInterface::updateMeshToGraphMappings(
    const std::vector<BlockIndex>& updated_blocks) {
  for (const auto& block : updated_blocks) {
    for (const auto& remap : msg_to_mesh_idx_->at(block)) {
      // TODO(Yun) some mesh vertex might not have graph index if part of the
      // mesh is disconnected and all contained within a block since we remove
      // degenerate faces
      if (msg_to_graph_idx_->at(block).count(remap.first)) {
        mesh_to_graph_idx_->insert(
            {remap.second, msg_to_graph_idx_->at(block).at(remap.first)});
      }
    }
  }
  return;
}

void MeshFrontendInterface::logGraphProcess(int duration,
                                            size_t num_indices,
                                            size_t num_edges) const {
  std::ofstream file;
  std::string filename = config.log_path + std::string("/mf_graph_log.csv");
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

void MeshFrontendInterface::logFullProcess(int duration) const {
  std::ofstream file;
  std::string filename = config.log_path + std::string("/mf_full_log.csv");
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
