/**
 * @file   alliez_desbrun.cpp
 * @brief  Demo for mesh compression
 * @author Yun Chang
 */
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <ros/ros.h>

#include "mesher_mapper/CommonFunctions.h"

using mesher_mapper::ConstructPolygonMeshMsg;
using mesher_mapper::ReadMeshFromPly;

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "load_mesh");
  pcl::PolygonMeshPtr input_mesh(new pcl::PolygonMesh);
  ReadMeshFromPly(argv[1], input_mesh);
  ros::NodeHandle n;
  ros::Publisher mesh_pub = n.advertise<pcl_msgs::PolygonMesh>("mesh", 10);

  ros::Rate loop_rate(10);

  pcl_msgs::PolygonMesh mesh_msg;
  pcl_conversions::moveFromPCL(*input_mesh, mesh_msg);

  while (ros::ok()) {
    mesh_msg.header.stamp = ros::Time::now();
    mesh_pub.publish(mesh_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
