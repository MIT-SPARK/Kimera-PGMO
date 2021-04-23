/**
 * @file   MeshFrontend.cpp
 * @brief  MeshFrontend class: process incoming voxblox meshes and sample it for
 * the mesh parts of the deformation graph
 * @author Yun Chang
 */
#include <chrono>

#include "kimera_pgmo/TriangleMeshIdStamped.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

#include "kimera_pgmo/MeshFrontend.h"

namespace kimera_pgmo {

MeshFrontend::MeshFrontend()
    : vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>),
      graph_vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>),
      initilized_log_(false) {}
MeshFrontend::~MeshFrontend() {}

// Initialize parameters, publishers, and subscribers
bool MeshFrontend::initialize(const ros::NodeHandle& n) {
  if (!loadParameters(n)) {
    ROS_ERROR("MeshFrontend: Failed to load parameters.");
  }

  if (!createPublishers(n)) {
    ROS_ERROR("MeshFrontend: Failed to create publishers.");
  }

  if (!registerCallbacks(n)) {
    ROS_ERROR("MeshFrontend: Failed to register callbacks.");
  }

  // Log header to file
  if (log_output_) {
    std::string log_file = log_path_ + std::string("/mesh_frontend_log.csv");
    logTiming(log_file);
    initilized_log_ = true;
  }

  ROS_INFO("Initialized MeshFrontend.");

  return true;
}

bool MeshFrontend::loadParameters(const ros::NodeHandle& n) {
  if (!n.getParam("horizon", time_horizon_)) return false;
  if (!n.getParam("robot_id", robot_id_)) return false;

  double mesh_resolution, d_graph_resolution;
  if (!n.getParam("output_mesh_resolution", mesh_resolution)) return false;
  if (!n.getParam("d_graph_resolution", d_graph_resolution)) return false;

  full_mesh_compression_.reset(new OctreeCompression(mesh_resolution));
  d_graph_compression_.reset(new OctreeCompression(d_graph_resolution));

  if (n.getParam("log_path", log_path_)) {
    n.getParam("log_output", log_output_);
    if (log_output_) {
      ROS_INFO("Logging output to: %s/mesh_frontend_log.csv",
               log_path_.c_str());
    }
  }

  return true;
}

bool MeshFrontend::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  full_mesh_pub_ =
      nl.advertise<kimera_pgmo::TriangleMeshIdStamped>("full_mesh", 1, false);
  simplified_mesh_pub_ = nl.advertise<mesh_msgs::TriangleMeshStamped>(
      "deformation_graph_mesh", 10, false);
  mesh_graph_pub_ = nl.advertise<pose_graph_tools::PoseGraph>(
      "mesh_graph_incremental", 30, false);
  return true;
}

bool MeshFrontend::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  voxblox_sub_ =
      nl.subscribe("voxblox_mesh", 20, &MeshFrontend::voxbloxCallback, this);
  return true;
}

void MeshFrontend::voxbloxCallback(const voxblox_msgs::Mesh::ConstPtr& msg) {
  processVoxbloxMesh(msg);

  // Publish partial and full mesh
  publishFullMesh(msg->header.stamp);
  publishSimplifiedMesh(msg->header.stamp);
  return;
}

// Creates partial mesh while updating the full mesh and also the last detected
// mesh blocks
void MeshFrontend::processVoxbloxMesh(const voxblox_msgs::Mesh::ConstPtr& msg) {
  // Start timer
  auto start = std::chrono::high_resolution_clock::now();

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mesh_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  boost::shared_ptr<std::vector<pcl::Vertices> > mesh_surfaces =
      boost::make_shared<std::vector<pcl::Vertices> >();

  // First prune the mesh blocks
  const double msg_time = msg->header.stamp.toSec();
  full_mesh_compression_->pruneStoredMesh(msg_time - time_horizon_);
  d_graph_compression_->pruneStoredMesh(msg_time - time_horizon_);

  // Iterate through the mesh blocks
  for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
    // For each mesh block
    // Convert it into a vertices - surfaces format and input to compressor
    // Full mesh will then be extracted from compressor
    // While mesh blocks are combined to build a partial mesh to be returned
    if (mesh_block.x.size() > 3) {
      // Convert to vertices and surfaces
      VoxbloxMeshBlockToPolygonMesh(
          mesh_block, msg->block_edge_length, mesh_vertices, mesh_surfaces);
    }
  }

  if (mesh_vertices->size() < 3 || mesh_surfaces->size() == 0) return;

  // Add to full mesh compressor
  try {
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
        new pcl::PointCloud<pcl::PointXYZRGBA>);
    boost::shared_ptr<std::vector<pcl::Vertices> > new_triangles =
        boost::make_shared<std::vector<pcl::Vertices> >();
    boost::shared_ptr<std::vector<size_t> > new_indices =
        boost::make_shared<std::vector<size_t> >();
    full_mesh_compression_->compressAndIntegrate(*mesh_vertices,
                                                 *mesh_surfaces,
                                                 std::move(new_vertices),
                                                 std::move(new_triangles),
                                                 std::move(new_indices),
                                                 msg_time);
    // Update the mesh vertices and surfaces for class variables
    full_mesh_compression_->getVertices(vertices_);
    full_mesh_compression_->getStoredPolygons(&triangles_);
  } catch (...) {
    ROS_ERROR("MeshFrontend: Failed to compress full mesh. ");
  }

  // Add to deformation graph mesh compressor

  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_graph_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  boost::shared_ptr<std::vector<pcl::Vertices> > new_graph_triangles =
      boost::make_shared<std::vector<pcl::Vertices> >();
  boost::shared_ptr<std::vector<size_t> > new_graph_indices =
      boost::make_shared<std::vector<size_t> >();
  try {
    d_graph_compression_->compressAndIntegrate(*mesh_vertices,
                                               *mesh_surfaces,
                                               new_graph_vertices,
                                               new_graph_triangles,
                                               new_graph_indices,
                                               msg_time);
    // Update the simplified mesh vertices and surfaces for class variables
    d_graph_compression_->getVertices(graph_vertices_);
    d_graph_compression_->getStoredPolygons(&graph_triangles_);
  } catch (...) {
    ROS_ERROR("MeshFrontend: Failed to simplify mesh for deformation graph. ");
  }
  std::vector<Edge> new_graph_edges;
  if (new_graph_indices->size() > 0 && new_graph_triangles->size() > 0) {
    // Add nodes and edges to graph
    new_graph_edges = simplified_mesh_graph_.addPointsAndSurfaces(
        *new_graph_indices.get(), *new_graph_triangles.get());
    // Publish edges and nodes
    last_mesh_graph_ = publishMeshGraph(
        new_graph_edges, *new_graph_indices.get(), msg->header);
  }

  // Stop timer and save
  auto stop = std::chrono::high_resolution_clock::now();
  auto spin_duration =
      std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
  // Log to file
  if (log_output_) {
    std::string log_file = log_path_ + std::string("/mesh_frontend_log.csv");
    logTiming(log_file,
              spin_duration.count(),
              new_graph_indices->size(),
              new_graph_edges.size());
  }

  return;
}

void MeshFrontend::publishFullMesh(const ros::Time& stamp) const {
  if (full_mesh_pub_.getNumSubscribers() == 0) return;
  // convert to triangle mesh msg
  mesh_msgs::TriangleMesh mesh_msg =
      kimera_pgmo::PolygonMeshToTriangleMeshMsg(*vertices_, triangles_);
  // publish
  kimera_pgmo::TriangleMeshIdStamped new_msg;
  new_msg.header.stamp = stamp;
  new_msg.header.frame_id = "world";
  new_msg.mesh = mesh_msg;
  new_msg.id = robot_id_;
  full_mesh_pub_.publish(new_msg);
  return;
}

void MeshFrontend::publishSimplifiedMesh(const ros::Time& stamp) const {
  if (simplified_mesh_pub_.getNumSubscribers() == 0) return;
  // convert to triangle mesh msg
  mesh_msgs::TriangleMesh mesh_msg = kimera_pgmo::PolygonMeshToTriangleMeshMsg(
      *graph_vertices_, graph_triangles_);

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
  for (auto e : new_edges) {
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
    pose_graph_msg.edges.push_back(pg_edge);
  }

  // Encode the new vertices as nodes
  for (auto n : new_indices) {
    pose_graph_tools::PoseGraphNode pg_node;
    pg_node.header = header;
    pg_node.robot_id = robot_id_;

    pg_node.key = n;

    gtsam::Point3 node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices_->at(n));
    pg_node.pose = GtsamToRos(gtsam::Pose3(gtsam::Rot3(), node_pos));

    // Add node to pose graph
    pose_graph_msg.nodes.push_back(pg_node);
  }

  // Publish if subscribed
  if (mesh_graph_pub_.getNumSubscribers() > 0) {
    mesh_graph_pub_.publish(pose_graph_msg);
  }

  return pose_graph_msg;
}

void MeshFrontend::logTiming(const std::string& filename,
                             const int& callback_duration,
                             const size_t& num_indices,
                             const size_t& num_edges) const {
  std::ofstream file;

  if (!initilized_log_) {
    file.open(filename);
    // file format
    file << "num-vertices,num-vertices-simplified,num-vertices-new,num-edges-"
            "new,cb-time(mu-s)\n";
    return;
  }

  file.open(filename, std::ofstream::out | std::ofstream::app);
  file << vertices_->size() << "," << graph_vertices_->size() << ","
       << num_indices << "," << num_edges << "," << callback_duration
       << std::endl;
  file.close();
  return;
}

}  // namespace kimera_pgmo
