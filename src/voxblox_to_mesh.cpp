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

#include "kimera_pgmo/CommonFunctions.h"

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
        "voxblox_mesh", 10, &VoxbloxToMeshMsg::voxbloxCallback, this);

    mesh_pub_ = nl.advertise<mesh_msgs::TriangleMeshStamped>(
        "converted_mesh", 10, true);
  }

  ~VoxbloxToMeshMsg() {}

 private:
  void voxbloxCallback(const voxblox_msgs::Mesh::ConstPtr& msg) {
    for (const voxblox_msgs::MeshBlock& mesh_block : msg->mesh_blocks) {
      BlockIndex idx(
          mesh_block.index[0], mesh_block.index[1], mesh_block.index[2]);
      pcl::PolygonMesh partial_mesh =
          kimera_pgmo ::VoxbloxMeshBlockToPolygonMesh(mesh_block,
                                                      msg->block_edge_length);
      mesh_blocks_[idx] = partial_mesh;
    }
    // Combine the partial meshes stored in the mesh blocks to get the full mesh
    pcl::PolygonMesh mesh;
    for (auto meshblock : mesh_blocks_) {
      mesh = kimera_pgmo ::CombineMeshes(mesh, meshblock.second);
    }
    mesh_msgs::TriangleMesh mesh_msg =
        kimera_pgmo ::PolygonMeshToTriangleMeshMsg(mesh);

    // publish
    mesh_msgs::TriangleMeshStamped new_msg;
    new_msg.header.stamp = msg->header.stamp;
    new_msg.header.frame_id = "world";
    new_msg.mesh = mesh_msg;
    mesh_pub_.publish(new_msg);
  }

  ros::Subscriber voxblox_sub_;
  ros::Publisher mesh_pub_;

  std::map<BlockIndex, pcl::PolygonMesh> mesh_blocks_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_to_mesh");
  ros::NodeHandle n;

  VoxbloxToMeshMsg voxblox_to_mesh(n);

  ros::spin();

  return 0;
}