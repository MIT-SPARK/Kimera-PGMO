/**
 * @file   VoxbloxProcessing.cpp
 * @brief  VoxbloxProcessing class: process incoming voxblox meshes
 * @author Yun Chang
 */
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
  double time_horizon_sec;
  if (!n.getParam("horizon", time_horizon_sec)) return false;
  time_horizon_ = ros::Duration(time_horizon_sec);

  return true;
}

bool VoxbloxProcessing::createPublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  full_mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("full_mesh", 1, false);
  partial_mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("partial_mesh", 1, false);
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

void VoxbloxProcessing::pruneStoredBlocks(const ros::Time& latest_time) {
  if (latest_time.toSec() - time_horizon_.toSec() < 0) return;
  for (auto it = mesh_block_last_detection_.cbegin();
       it != mesh_block_last_detection_.cend();) {
    if (it->second < latest_time - time_horizon_) {
      mesh_block_vertices_.erase(it->first);
      it = mesh_block_last_detection_.erase(it);
    } else {
      ++it;
    }
  }
}

// Creates partial mesh while updating the full mesh and also the last detected
// mesh blocks
pcl::PolygonMesh VoxbloxProcessing::processVoxbloxMesh(
    const voxblox_msgs::Mesh::ConstPtr& msg) {
  // Timestamp of voxblox mesh
  ros::Time msg_timestamp = msg->header.stamp;
  // Initiate the partial mesh to be returned
  pcl::PolygonMesh partial_mesh;

  // First prune the mesh blocks
  pruneStoredBlocks(msg_timestamp);

  // Iterate through the mesh blocks
  for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
    if (mesh_block.x.size() > 0) {
      BlockIndex idx(
          mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);

      pcl::PolygonMesh meshblock_mesh;
      // Check if block in queue
      std::map<BlockIndex, std::vector<size_t>>::iterator it =
          mesh_block_vertices_.find(idx);
      // Keep track of indices of vertex corresponding to mesh block
      std::vector<size_t> block_indices;
      if (it == mesh_block_vertices_.end()) {
        // new block
        std::vector<size_t> orig_mesh_block;  // empty vector to make do
        meshblock_mesh = UpdateMeshFromVoxbloxMeshBlock(mesh_block,
                                                        msg->block_edge_length,
                                                        vertices_,
                                                        &triangles_,
                                                        orig_mesh_block,
                                                        &block_indices);
      } else {
        // previously seen block
        meshblock_mesh =
            UpdateMeshFromVoxbloxMeshBlock(mesh_block,
                                           msg->block_edge_length,
                                           vertices_,
                                           &triangles_,
                                           mesh_block_vertices_[idx],
                                           &block_indices);
      }
      // track vertex indices of full mesh associated with block
      mesh_block_vertices_[idx] = block_indices;
      mesh_block_last_detection_[idx] = msg_timestamp;

      // Add to partial mesh
      partial_mesh = CombineMeshes(partial_mesh, meshblock_mesh);
    }
  }

  // Return partial mesh
  return partial_mesh;
}

void VoxbloxProcessing::publishPartialMesh(const pcl::PolygonMesh& mesh,
                                           const ros::Time& stamp) const {
  // convert to triangle mesh msg
  mesh_msgs::TriangleMesh mesh_msg =
      kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh);
  // publish
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = stamp;
  new_msg.header.frame_id = "world";
  new_msg.mesh = mesh_msg;
  partial_mesh_pub_.publish(new_msg);
}

void VoxbloxProcessing::publishFullMesh(const ros::Time& stamp) const {
  // convert to triangle mesh msg
  mesh_msgs::TriangleMesh mesh_msg =
      kimera_pgmo::PolygonMeshToTriangleMeshMsg(*vertices_, triangles_);
  // publish
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = stamp;
  new_msg.header.frame_id = "world";
  new_msg.mesh = mesh_msg;
  full_mesh_pub_.publish(new_msg);
}

}  // namespace kimera_pgmo
