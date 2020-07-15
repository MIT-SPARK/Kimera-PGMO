/**
 * @file   voxblox_to_mesh.cpp
 * @brief  node to convert voxblox msg to mesh msg (last mesh block)
 * @author Yun Chang
 */

#include <map>
#include "ros/ros.h"

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
        "voxblox_mesh", 30, &VoxbloxToMeshMsg::voxbloxCallback, this);

    mesh_pub_ =
        nl.advertise<mesh_msgs::TriangleMeshStamped>("converted_mesh", 5, true);
  }

  ~VoxbloxToMeshMsg() {}

 private:
  void voxbloxCallback(const voxblox_msgs::Mesh::ConstPtr& msg) {

    for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
      BlockIndex idx(
          mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);
      // convert voxblox mesh block to polygon mesh
      pcl::PolygonMesh partial_mesh =
          kimera_pgmo::VoxbloxMeshBlockToPolygonMesh(mesh_block,
                                                     msg->block_edge_length);
      std::vector<size_t> block_indices;
      // Check if block previously populated
      std::map<BlockIndex, std::vector<size_t>>::iterator it =
          blocks_.find(idx);
      if (it == blocks_.end()) {
        // new block
        std::vector<size_t> orig_mesh_block;  // empty vector to make do
        mesh_ = kimera_pgmo::CombineMeshes(
            mesh_, partial_mesh, orig_mesh_block, &block_indices);
      } else {
        // previously seen block
        mesh_ = kimera_pgmo::CombineMeshes(
            mesh_, partial_mesh, blocks_[idx], &block_indices);
      }
      // track vertex indices of full mesh associated with block
      blocks_[idx] = block_indices;
    }

    // convert to triangle mesh msg
    mesh_msgs::TriangleMesh mesh_msg =
        kimera_pgmo::PolygonMeshToTriangleMeshMsg(mesh_);
    // publish
    mesh_msgs::TriangleMeshStamped new_msg;
    new_msg.header.stamp = msg->header.stamp;
    new_msg.header.frame_id = "world";
    new_msg.mesh = mesh_msg;
    mesh_pub_.publish(new_msg);
  }

  ros::Subscriber voxblox_sub_;
  ros::Publisher mesh_pub_;

  // keep track of the indices in the cloud
  // representing the vertices of the mesh its corresponding block
  std::map<BlockIndex, std::vector<size_t>> blocks_;
  pcl::PolygonMesh mesh_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_to_mesh");
  ros::NodeHandle n;

  VoxbloxToMeshMsg voxblox_to_mesh(n);

  ros::spin();

  return 0;
}