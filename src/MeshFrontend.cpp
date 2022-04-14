/**
 * @file   MeshFrontend.cpp
 * @brief  MeshFrontend class: process incoming voxblox meshes and sample it for
 * the mesh parts of the deformation graph
 * @author Yun Chang
 */
#include <chrono>

#include "kimera_pgmo/KimeraPgmoMesh.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

#include "kimera_pgmo/MeshFrontend.h"
#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/compression/VoxbloxCompression.h"
#include "kimera_pgmo/compression/VoxelClearingCompression.h"

namespace kimera_pgmo {

MeshFrontend::MeshFrontend()
    : vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>),
      vertex_stamps_(new std::vector<ros::Time>),
      graph_vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>),
      triangles_(new std::vector<pcl::Vertices>),
      graph_triangles_(new std::vector<pcl::Vertices>),
      vxblx_msg_to_graph_idx_(new VoxbloxIndexMapping),
      vxblx_msg_to_mesh_idx_(new VoxbloxIndexMapping),
      mesh_to_graph_idx_(new IndexMapping),
      init_graph_log_(false),
      init_full_log_(false),
      voxblox_queue_size_(20),
      b_track_mesh_graph_mapping_(true),
      voxblox_update_called_(false) {}

MeshFrontend::~MeshFrontend() {}

// Initialize parameters, publishers, and subscribers
bool MeshFrontend::initialize(const ros::NodeHandle& n,
                              bool should_register_callbacks) {
  if (!loadParameters(n)) {
    ROS_ERROR("MeshFrontend: Failed to load parameters.");
  }

  if (!createPublishers(n)) {
    ROS_ERROR("MeshFrontend: Failed to create publishers.");
  }

  if (should_register_callbacks) {
    if (!registerCallbacks(n)) {
      ROS_ERROR("MeshFrontend: Failed to register callbacks.");
    }
  } else {
    ROS_WARN("MeshFrontend: Skipping callback registration");
  }

  // Log header to file
  if (log_output_) {
    logGraphProcess();
    logFullProcess();
    init_graph_log_ = true;
    init_full_log_ = true;
  }

  last_full_compression_stamp_ = ros::Time::now().toNSec();

  ROS_INFO("Initialized MeshFrontend.");

  return true;
}

bool MeshFrontend::loadParameters(const ros::NodeHandle& n) {
  if (!n.getParam("horizon", time_horizon_)) return false;
  if (!n.getParam("robot_id", robot_id_)) return false;

  double mesh_resolution, d_graph_resolution;
  if (!n.getParam("output_mesh_resolution", mesh_resolution)) return false;
  if (!n.getParam("d_graph_resolution", d_graph_resolution)) return false;

  int full_compression_method;
  if (!n.getParam("full_compression_method", full_compression_method)) {
    return false;
  }

  int graph_compression_method;
  if (!n.getParam("graph_compression_method", graph_compression_method)) {
    return false;
  }

  n.getParam("track_mesh_graph_mapping", b_track_mesh_graph_mapping_);

  n.getParam("voxblox_queue_size", voxblox_queue_size_);

  // 0 for octree, 1 for voxblox
  if (graph_compression_method == 0) {
    d_graph_compression_.reset(new OctreeCompression(d_graph_resolution));
  } else if (graph_compression_method == 1) {
    d_graph_compression_.reset(new VoxbloxCompression(d_graph_resolution));
  } else {
    ROS_ERROR("Invalid compression option for deformation graph. ");
    return false;
  }

  // 0 for octree, 1 for voxblox, 2 for voxel clearing
  if (full_compression_method == 0) {
    full_mesh_compression_.reset(new OctreeCompression(mesh_resolution));
  } else if (full_compression_method == 1) {
    full_mesh_compression_.reset(new VoxbloxCompression(mesh_resolution));
  } else if (full_compression_method == 2) {
    full_mesh_compression_.reset(new VoxelClearingCompression(mesh_resolution));
  } else {
    ROS_ERROR("Invalid full mesh compression option. ");
    return false;
  }

  if (n.getParam("log_path", log_path_)) {
    n.getParam("log_output", log_output_);
    if (log_output_) {
      ROS_INFO("Logging output to: %s", log_path_.c_str());
    }
  }

  return true;
}

bool MeshFrontend::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  full_mesh_pub_ =
      nl.advertise<kimera_pgmo::KimeraPgmoMesh>("full_mesh", 1, false);
  simplified_mesh_pub_ = nl.advertise<mesh_msgs::TriangleMeshStamped>(
      "deformation_graph_mesh", 10, false);
  mesh_graph_pub_ = nl.advertise<pose_graph_tools::PoseGraph>(
      "mesh_graph_incremental", 100, true);
  return true;
}

bool MeshFrontend::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  voxblox_sub_ = nl.subscribe("voxblox_mesh",
                              voxblox_queue_size_,
                              &MeshFrontend::voxbloxCallback,
                              this);
  return true;
}

void MeshFrontend::voxbloxCallback(const voxblox_msgs::Mesh::ConstPtr& msg) {
  // Start compression threads
  std::thread full_mesh_thread(&MeshFrontend::fullMeshUpdate, this, msg);
  std::thread graph_mesh_thread(&MeshFrontend::graphMeshUpdate, this, msg);

  latest_blocks_.clear();
  for (const auto& mesh_block : msg->mesh_blocks) {
    const voxblox::BlockIndex block_index(
        mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);
    latest_blocks_.push_back(block_index);
  }

  full_mesh_thread.join();
  graph_mesh_thread.join();

  if (b_track_mesh_graph_mapping_) {
    updateMeshToGraphMappings(latest_blocks_);
  }
  publishFullMesh();

  voxblox_update_called_ = true;
  return;
}

void MeshFrontend::fullMeshUpdate(const voxblox_msgs::Mesh::ConstPtr& msg) {
  processVoxbloxMeshFull(msg);
  last_full_compression_stamp_ = msg->header.stamp.toNSec();
  return;
}

void MeshFrontend::graphMeshUpdate(const voxblox_msgs::Mesh::ConstPtr& msg) {
  processVoxbloxMeshGraph(msg);
  publishSimplifiedMesh(msg->header.stamp);
  return;
}

// Update full mesh
void MeshFrontend::processVoxbloxMeshFull(
    const voxblox_msgs::Mesh::ConstPtr& msg) {
  // First prune the mesh blocks
  const double msg_time = msg->header.stamp.toSec();
  full_mesh_compression_->pruneStoredMesh(msg_time - time_horizon_);

  // Add to full mesh compressor
  auto f_comp_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_triangles =
      std::make_shared<std::vector<pcl::Vertices> >();
  std::shared_ptr<std::vector<size_t> > new_indices =
      std::make_shared<std::vector<size_t> >();
  full_mesh_compression_->compressAndIntegrate(*msg,
                                               new_vertices,
                                               new_triangles,
                                               new_indices,
                                               vxblx_msg_to_mesh_idx_,
                                               msg_time);

  auto f_comp_stop = std::chrono::high_resolution_clock::now();
  auto f_comp_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      f_comp_stop - f_comp_start);

  { // start critical section
    std::unique_lock<std::mutex> lock(full_mutex_);
    // Update the mesh vertices and surfaces for class variables
    full_mesh_compression_->getVertices(vertices_);
    full_mesh_compression_->getStoredPolygons(triangles_);
    full_mesh_compression_->getVertexStamps(vertex_stamps_);
    assert(vertex_stamps_.size() == vertices_.size());
    // save the active indices
    active_indices_ = full_mesh_compression_->getActiveVerticesIndex();
    invalid_indices_ = full_mesh_compression_->getInvalidIndices();
    if (log_output_) {
      logFullProcess(f_comp_duration.count());
    }
  }  // end critical section
  return;
}

void MeshFrontend::clearArchivedMeshFull(const voxblox_msgs::Mesh& msg) {
  full_mesh_compression_->clearArchivedBlocks(msg);
}

// Creates and update graph mesh and publish mesh graph
void MeshFrontend::processVoxbloxMeshGraph(
    const voxblox_msgs::Mesh::ConstPtr& msg) {
  // First prune the mesh blocks
  const double msg_time = msg->header.stamp.toSec();
  d_graph_compression_->pruneStoredMesh(msg_time - time_horizon_);

  // Add to deformation graph mesh compressor
  auto g_comp_start = std::chrono::high_resolution_clock::now();
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_graph_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::vector<pcl::Vertices> > new_graph_triangles =
      std::make_shared<std::vector<pcl::Vertices> >();
  std::shared_ptr<std::vector<size_t> > new_graph_indices =
      std::make_shared<std::vector<size_t> >();

  d_graph_compression_->compressAndIntegrate(*msg,
                                             new_graph_vertices,
                                             new_graph_triangles,
                                             new_graph_indices,
                                             vxblx_msg_to_graph_idx_,
                                             msg_time);

  auto g_comp_stop = std::chrono::high_resolution_clock::now();
  auto g_comp_duration = std::chrono::duration_cast<std::chrono::microseconds>(
      g_comp_stop - g_comp_start);

  {  // start critical section
    std::unique_lock<std::mutex> lock(graph_mutex_);
    // Update the simplified mesh vertices and surfaces for class variables
    d_graph_compression_->getVertices(graph_vertices_);
    d_graph_compression_->getStoredPolygons(graph_triangles_);

    std::vector<Edge> new_graph_edges;
    if (new_graph_indices->size() > 0 && new_graph_triangles->size() > 0) {
      // Add nodes and edges to graph
      new_graph_edges = simplified_mesh_graph_.addPointsAndSurfaces(
          *new_graph_indices.get(), *new_graph_triangles.get());
    }

    // Publish edges and nodes
    last_mesh_graph_ = publishMeshGraph(
        new_graph_edges, *new_graph_indices.get(), msg->header);

    if (log_output_) {
      logGraphProcess(g_comp_duration.count(),
                      new_graph_indices->size(),
                      new_graph_edges.size());
    }
  }
  return;
}

void MeshFrontend::updateMeshToGraphMappings(
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

void MeshFrontend::publishFullMesh() const {
  if (full_mesh_pub_.getNumSubscribers() == 0) return;
  if (vertices_->size() == 0) return;
  // convert to triangle mesh msg
  KimeraPgmoMesh mesh_msg =
      kimera_pgmo::PolygonMeshToPgmoMeshMsg(robot_id_,
                                            *vertices_,
                                            *triangles_,
                                            *vertex_stamps_,
                                            "world",
                                            *mesh_to_graph_idx_);
  // publish
  full_mesh_pub_.publish(mesh_msg);
  return;
}

void MeshFrontend::publishSimplifiedMesh(const ros::Time& stamp) const {
  if (simplified_mesh_pub_.getNumSubscribers() == 0) return;
  // convert to triangle mesh msg
  mesh_msgs::TriangleMesh mesh_msg = kimera_pgmo::PolygonMeshToTriangleMeshMsg(
      *graph_vertices_, *graph_triangles_);

  // Create msg
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = stamp;
  new_msg.header.frame_id = "world";
  new_msg.mesh = mesh_msg;
  simplified_mesh_pub_.publish(new_msg);
  return;
}

pose_graph_tools::PoseGraph MeshFrontend::publishMeshGraph(
    const std::vector<Edge>& new_edges,
    const std::vector<size_t>& new_indices,
    const std_msgs::Header& header) const {
  // Create message
  pose_graph_tools::PoseGraph pose_graph_msg;
  pose_graph_msg.header = header;

  // Encode the edges as factors
  pose_graph_msg.edges.resize(new_edges.size());
  for (size_t i = 0; i < new_edges.size(); i++) {
    const Edge& e = new_edges[i];
    pose_graph_tools::PoseGraphEdge pg_edge;
    pg_edge.header = header;

    const size_t& from_node = e.first;
    const size_t& to_node = e.second;

    pg_edge.robot_from = robot_id_;
    pg_edge.robot_to = robot_id_;
    pg_edge.key_from = from_node;
    pg_edge.key_to = to_node;

    gtsam::Point3 from_node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices_->at(from_node));
    gtsam::Point3 to_node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices_->at(to_node));
    pg_edge.pose =
        GtsamToRos(gtsam::Pose3(gtsam::Rot3(), to_node_pos - from_node_pos));

    pg_edge.type = pose_graph_tools::PoseGraphEdge::MESH;

    // Add edge to pose graph
    pose_graph_msg.edges[i] = pg_edge;
  }

  // Encode the new vertices as nodes
  pose_graph_msg.nodes.resize(new_indices.size());
  for (size_t i = 0; i < new_indices.size(); i++) {
    const size_t& n = new_indices[i];
    pose_graph_tools::PoseGraphNode pg_node;
    pg_node.header = header;
    pg_node.robot_id = robot_id_;

    pg_node.key = n;

    gtsam::Point3 node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices_->at(n));
    pg_node.pose = GtsamToRos(gtsam::Pose3(gtsam::Rot3(), node_pos));

    // Add node to pose graph
    pose_graph_msg.nodes[i] = pg_node;
  }

  // Publish if subscribed
  if (mesh_graph_pub_.getNumSubscribers() > 0) {
    mesh_graph_pub_.publish(pose_graph_msg);
  }

  return pose_graph_msg;
}

void MeshFrontend::logGraphProcess(const int& duration,
                                   const size_t& num_indices,
                                   const size_t& num_edges) const {
  std::ofstream file;
  std::string filename = log_path_ + std::string("/mf_graph_log.csv");
  if (!init_graph_log_) {
    file.open(filename);
    // file format
    file << "num-vertices-simplified,num-vertices-new,num-edges-new,compress-"
            "time(mu-s)\n";
    return;
  }

  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << graph_vertices_->size() << "," << num_indices << "," << num_edges
       << "," << duration << std::endl;
  file.close();
  return;
}

void MeshFrontend::logFullProcess(const int& duration) const {
  std::ofstream file;
  std::string filename = log_path_ + std::string("/mf_full_log.csv");
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
