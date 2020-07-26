/**
 * @file   voxblox_to_mesh.cpp
 * @brief  node to convert voxblox msg to mesh msg (last mesh block)
 * @author Yun Chang
 */

#include <ros/ros.h>
#include <map>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/PolygonMesh.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/utils/CommonFunctions.h"

// Define BlockIndex as used in voxblox
typedef Eigen::Matrix<int, 3, 1> BlockIndex;
namespace std {
template <>
struct less<BlockIndex> {
  bool operator()(const BlockIndex& a, const BlockIndex& b) const {
    return std::lexicographical_compare(
        a.data(), a.data() + a.size(), b.data(), b.data() + b.size());
  }
};
}  // namespace std

class VoxbloxToMeshMsg {
 public:
  VoxbloxToMeshMsg(const ros::NodeHandle& n) {
    ROS_INFO("Started Voxblox to Mesh msg converter.");
    ros::NodeHandle nl(n);
    voxblox_sub_ = nl.subscribe(
        "voxblox_mesh", 20, &VoxbloxToMeshMsg::voxbloxCallback, this);

    mesh_pub_ = nl.advertise<mesh_msgs::TriangleMeshStamped>(
        "converted_mesh", 1, false);

    // start timer
    update_timer_ = nl.createTimer(
        ros::Duration(1.0), &VoxbloxToMeshMsg::ProcessTimerCallback, this);
  }

  ~VoxbloxToMeshMsg() {}

 private:
  void voxbloxCallback(const voxblox_msgs::Mesh::ConstPtr& msg) {
    block_edge_length_ = msg->block_edge_length;
    last_stamp_ = msg->header.stamp;
    for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
      if (mesh_block.x.size() > 0) {
        BlockIndex idx(
            mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);

        mesh_blocks_[idx] = mesh_block;
      }
    }
  }

  void updateMesh() {
    // copy mesh blocks and clear
    std::map<BlockIndex, voxblox_msgs::MeshBlock> new_mesh_blocks =
        mesh_blocks_;
    mesh_blocks_.clear();

    for (auto idx_mesh_block : new_mesh_blocks) {
      BlockIndex idx = idx_mesh_block.first;
      voxblox_msgs::MeshBlock mesh_block = idx_mesh_block.second;
      std::vector<size_t> block_indices;
      // Check if block previously populated
      std::map<BlockIndex, std::vector<size_t>>::iterator it =
          blocks_.find(idx);
      if (it == blocks_.end()) {
        // new block
        std::vector<size_t> orig_mesh_block;  // empty vector to make do
        mesh_ = kimera_pgmo::UpdateMesh(mesh_,
                                        mesh_block,
                                        block_edge_length_,
                                        orig_mesh_block,
                                        &block_indices);
      } else {
        // previously seen block
        mesh_ = kimera_pgmo::UpdateMesh(mesh_,
                                        mesh_block,
                                        block_edge_length_,
                                        blocks_[idx],
                                        &block_indices);
      }
      // track vertex indices of full mesh associated with block
      blocks_[idx] = block_indices;
    }
  }

  void publishMesh() const {
    // convert to triangle mesh msg
    mesh_msgs::TriangleMesh mesh_msg =
        kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh_);
    // publish
    mesh_msgs::TriangleMeshStamped new_msg;
    new_msg.header.stamp = last_stamp_;
    new_msg.header.frame_id = "world";
    new_msg.mesh = mesh_msg;
    mesh_pub_.publish(new_msg);
  }

  // Timer callback
  void ProcessTimerCallback(const ros::TimerEvent& ev) {
    if (mesh_blocks_.size() > 0) {
      updateMesh();
      publishMesh();
    }
  }

  // Class arguments
  ros::Subscriber voxblox_sub_;
  ros::Publisher mesh_pub_;
  ros::Timer update_timer_;

  ros::Time last_stamp_;

  float block_edge_length_;
  std::map<BlockIndex, voxblox_msgs::MeshBlock> mesh_blocks_;
  // keep track of the indices in the cloud
  // representing the vertices of the mesh its corresponding block
  std::map<BlockIndex, std::vector<size_t>> blocks_;
  pcl::PolygonMesh mesh_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_to_mesh");
  ros::NodeHandle n;

  // start async spinner
  ros::AsyncSpinner spinner(0);
  spinner.start();

  VoxbloxToMeshMsg voxblox_to_mesh(n);

  ros::waitForShutdown();

  return 0;
}