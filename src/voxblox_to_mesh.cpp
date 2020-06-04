/**
 * @file   voxblox_to_mesh.cpp
 * @brief  node to convert voxblox msg to mesh msg (last mesh block)
 * @author Yun Chang
 */

#include "ros/ros.h"

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/PolygonMesh.h>
#include <voxblox_msgs/Mesh.h>

#include "mesher_mapper/CommonFunctions.h"

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
    pcl::PolygonMesh mesh = mesher_mapper::VoxbloxGetLastMeshBlock(msg);
    mesh_msgs::TriangleMesh mesh_msg =
        mesher_mapper::PolygonMeshToTriangleMeshMsg(mesh);

    // publish
    mesh_msgs::TriangleMeshStamped new_msg;
    new_msg.header.stamp = ros::Time::now();
    new_msg.header.frame_id = "world";
    new_msg.mesh = mesh_msg;
    mesh_pub_.publish(new_msg);
  }

  ros::Subscriber voxblox_sub_;
  ros::Publisher mesh_pub_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "voxblox_to_mesh");
  ros::NodeHandle n;

  VoxbloxToMeshMsg voxblox_to_mesh(n);

  ros::spin();

  return 0;
}