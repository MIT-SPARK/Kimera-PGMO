/**
 * @file   MeshFrontend.cpp
 * @brief  MeshFrontend class: process incoming voxblox meshes
 * @author Yun Chang
 */
#include "kimera_pgmo/TriangleMeshIdStamped.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

#include "kimera_pgmo/MeshFrontend.h"

namespace kimera_pgmo {

MeshFrontend::MeshFrontend()
    : vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>),
      graph_vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>) {}
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

  return true;
}

bool MeshFrontend::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  full_mesh_pub_ =
      nl.advertise<kimera_pgmo::TriangleMeshIdStamped>("full_mesh", 1, false);
  simplified_mesh_pub_ = nl.advertise<mesh_msgs::TriangleMeshStamped>(
      "deformation_graph_mesh", 10, false);
  mesh_graph_pub_ =
      nl.advertise<pose_graph_tools::PoseGraph>("mesh_graph", 10, false);
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
}

// Creates partial mesh while updating the full mesh and also the last detected
// mesh blocks
void MeshFrontend::processVoxbloxMesh(const voxblox_msgs::Mesh::ConstPtr& msg) {
  // Initiate the partial mesh to be returned
  pcl::PolygonMesh partial_mesh;

  // First prune the mesh blocks
  const double msg_time = msg->header.stamp.toSec();
  full_mesh_compression_->pruneStoredMesh(msg_time - time_horizon_);
  d_graph_compression_->pruneStoredMesh(msg_time - time_horizon_);

  std::vector<Edge> new_edges;
  std::vector<size_t> new_indices;

  // Iterate through the mesh blocks
  for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
    // For each mesh block
    // Convert it into a vertices - surfaces format and input to compressor
    // Full mesh will then be extracted from compressor
    // While mesh blocks are combined to build a partial mesh to be returned
    if (mesh_block.x.size() > 0) {
      pcl::PolygonMesh meshblock_mesh;
      // Convert to vertices and surfaces
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mesh_block_vertices(
          new pcl::PointCloud<pcl::PointXYZRGBA>);
      std::vector<pcl::Vertices> mesh_block_surfaces;
      VoxbloxMeshBlockToPolygonMesh(mesh_block,
                                    msg->block_edge_length,
                                    mesh_block_vertices,
                                    &mesh_block_surfaces);

      // Add to full mesh compressor
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
          new pcl::PointCloud<pcl::PointXYZRGBA>);
      std::vector<pcl::Vertices> new_triangles;
      std::vector<size_t> new_indices;
      // Pass for mesh compression
      full_mesh_compression_->compressAndIntegrate(*mesh_block_vertices,
                                                   mesh_block_surfaces,
                                                   new_vertices,
                                                   &new_triangles,
                                                   &new_indices,
                                                   msg_time);

      // Add to deformation graph mesh compressor
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_graph_vertices(
          new pcl::PointCloud<pcl::PointXYZRGBA>);
      std::vector<pcl::Vertices> new_graph_triangles;
      std::vector<size_t> new_graph_indices;

      d_graph_compression_->compressAndIntegrate(*mesh_block_vertices,
                                                 mesh_block_surfaces,
                                                 new_graph_vertices,
                                                 &new_graph_triangles,
                                                 &new_graph_indices,
                                                 msg_time);

      const std::vector<Edge>& block_new_edges =
          simplified_mesh_graph_.addPointsAndSurfaces(new_graph_indices,
                                                      new_graph_triangles);
      new_edges.insert(
          new_edges.end(), block_new_edges.begin(), block_new_edges.end());
      new_indices.insert(new_indices.end(),
                         new_graph_indices.begin(),
                         new_graph_indices.end());
    }
  }

  // Update the mesh vertices and surfaces
  full_mesh_compression_->getVertices(vertices_);
  full_mesh_compression_->getStoredPolygons(&triangles_);
  d_graph_compression_->getVertices(graph_vertices_);
  d_graph_compression_->getStoredPolygons(&graph_triangles_);

  last_mesh_graph_ = publishMeshGraph(new_edges, new_indices, msg->header);

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
}

pose_graph_tools::PoseGraph MeshFrontend::publishMeshGraph(
    const std::vector<Edge>& new_edges,
    const std::vector<Vertex>& new_indices,
    const std_msgs::Header& header) const {
  // Create message
  pose_graph_tools::PoseGraph pose_graph_msg;
  pose_graph_msg.header = header;

  // Encode the edges as factors
  for (auto e : new_edges) {
    pose_graph_tools::PoseGraphEdge pg_edge;
    pg_edge.header = header;
    const size_t& to_node = e.first;
    const size_t& from_node = e.second;
    pg_edge.robot_from = robot_id_;
    pg_edge.robot_to = robot_id_;
    pg_edge.key_from = from_node;
    pg_edge.key_to = to_node;
    gtsam::Point3 to_node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices_->at(to_node));
    gtsam::Point3 from_node_pos =
        PclToGtsam<pcl::PointXYZRGBA>(graph_vertices_->at(from_node));
    pg_edge.pose =
        GtsamToRos(gtsam::Pose3(gtsam::Rot3(), from_node_pos - to_node_pos));
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

}  // namespace kimera_pgmo
