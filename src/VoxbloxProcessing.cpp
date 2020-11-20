/**
 * @file   VoxbloxProcessing.cpp
 * @brief  VoxbloxProcessing class: process incoming voxblox meshes
 * @author Yun Chang
 */
#include "kimera_pgmo/TriangleMeshIdStamped.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

#include "kimera_pgmo/VoxbloxProcessing.h"

namespace kimera_pgmo {

VoxbloxProcessing::VoxbloxProcessing()
    : vertices_(new pcl::PointCloud<pcl::PointXYZRGBA>) {}
VoxbloxProcessing::~VoxbloxProcessing() {}

// Initialize parameters, publishers, and subscribers
bool VoxbloxProcessing::initialize(const ros::NodeHandle& n) {
  if (!loadParameters(n)) {
    ROS_ERROR("VoxbloxProcessing: Failed to load parameters.");
  }

  if (!createPublishers(n)) {
    ROS_ERROR("VoxbloxProcessing: Failed to create publishers.");
  }

  if (!registerCallbacks(n)) {
    ROS_ERROR("VoxbloxProcessing: Failed to register callbacks.");
  }

  ROS_INFO("Initialized VoxbloxProcessing.");

  return true;
}

bool VoxbloxProcessing::loadParameters(const ros::NodeHandle& n) {
  if (!n.getParam("horizon", time_horizon_)) return false;
  if (!n.getParam("robot_id", robot_id_)) return false;

  double mesh_resolution;
  if (!n.getParam("output_mesh_resolution", mesh_resolution)) return false;

  compression_.reset(new OctreeCompression(mesh_resolution));

  return true;
}

bool VoxbloxProcessing::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  full_mesh_pub_ =
      nl.advertise<kimera_pgmo::TriangleMeshIdStamped>("full_mesh", 1, false);
  partial_mesh_pub_ = nl.advertise<kimera_pgmo::TriangleMeshIdStamped>(
      "partial_mesh", 10, false);
  return true;
}

bool VoxbloxProcessing::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  voxblox_sub_ = nl.subscribe(
      "voxblox_mesh", 20, &VoxbloxProcessing::voxbloxCallback, this);
  return true;
}

void VoxbloxProcessing::voxbloxCallback(
    const voxblox_msgs::Mesh::ConstPtr& msg) {
  pcl::PolygonMesh partial_mesh = processVoxbloxMesh(msg);

  // Publish partial and full mesh
  if (partial_mesh_pub_.getNumSubscribers() > 0)
    publishPartialMesh(partial_mesh, msg->header.stamp);
  if (full_mesh_pub_.getNumSubscribers() > 0)
    publishFullMesh(msg->header.stamp);
}

// Creates partial mesh while updating the full mesh and also the last detected
// mesh blocks
pcl::PolygonMesh VoxbloxProcessing::processVoxbloxMesh(
    const voxblox_msgs::Mesh::ConstPtr& msg) {
  // Timestamp of voxblox mesh
  const ros::Time& msg_timestamp = msg->header.stamp;
  // Initiate the partial mesh to be returned
  pcl::PolygonMesh partial_mesh;

  // First prune the mesh blocks
  const double& msg_time = msg_timestamp.toSec();
  compression_->pruneStoredMesh(msg_time - time_horizon_);

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

      // Add to compressor

      // Note these following values are arguments compress method
      // but unsued
      pcl::PointCloud<pcl::PointXYZRGBA>::Ptr new_vertices(
          new pcl::PointCloud<pcl::PointXYZRGBA>);
      std::vector<pcl::Vertices> new_triangles;
      std::vector<size_t> new_indices;
      // Pass for mesh compression
      compression_->compressAndIntegrate(*mesh_block_vertices,
                                         mesh_block_surfaces,
                                         new_vertices,
                                         &new_triangles,
                                         &new_indices,
                                         msg_time);

      // Mesh block mesh
      pcl::toPCLPointCloud2(*mesh_block_vertices, meshblock_mesh.cloud);
      meshblock_mesh.polygons = mesh_block_surfaces;

      // Add to partial mesh
      partial_mesh = CombineMeshes(partial_mesh, meshblock_mesh);
    }
  }

  // Update the full mesh vertices and surfaces
  compression_->getVertices(vertices_);
  compression_->getStoredPolygons(&triangles_);

  // Return partial mesh
  return partial_mesh;
}

void VoxbloxProcessing::publishPartialMesh(const pcl::PolygonMesh& mesh,
                                           const ros::Time& stamp) const {  
  // publish
  kimera_pgmo::TriangleMeshIdStamped new_msg;
  new_msg.header.stamp = stamp;
  new_msg.header.frame_id = "world";
  // convert to triangle mesh msg
  new_msg.mesh = kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh);
  new_msg.id = robot_id_;
  partial_mesh_pub_.publish(new_msg);
}

void VoxbloxProcessing::publishFullMesh(const ros::Time& stamp) const {
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

}  // namespace kimera_pgmo
