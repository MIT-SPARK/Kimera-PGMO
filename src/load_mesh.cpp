/**
 * @file   alliez_desbrun.cpp
 * @brief  Demo for mesh compression
 * @author Yun Chang
 */
#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include "kimera_pgmo/utils/CommonFunctions.h"

using kimera_pgmo::PolygonMeshToTriangleMeshMsg;
using kimera_pgmo::ReadMeshFromPly;

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "load_mesh");
  pcl::PolygonMeshPtr input_mesh(new pcl::PolygonMesh);
  ReadMeshFromPly(argv[1], input_mesh);
  ros::NodeHandle n;
  ros::Publisher mesh_pub =
      n.advertise<mesh_msgs::TriangleMeshStamped>("mesh", 10);

  ros::Rate loop_rate(10);

  mesh_msgs::TriangleMesh mesh_msg = PolygonMeshToTriangleMeshMsg(*input_mesh);

  while (ros::ok()) {
    mesh_msgs::TriangleMeshStamped new_msg;
    new_msg.header.stamp = ros::Time::now();
    new_msg.header.frame_id = "world";
    new_msg.mesh = mesh_msg;
    mesh_pub.publish(new_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
