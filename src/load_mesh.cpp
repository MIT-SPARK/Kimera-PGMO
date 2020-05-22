/**
 * @file   alliez_desbrun.cpp
 * @brief  Demo for mesh compression
 * @author Yun Chang
 */
#include <colorized_mesh_display/ColorizedMesh.h>
#include <colorized_mesh_display/ColorizedMeshStamped.h>
#include <colorized_mesh_display/utils.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>
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
  ros::Publisher mesh_pub =
      n.advertise<colorized_mesh_display::ColorizedMeshStamped>("mesh", 10);

  ros::Rate loop_rate(10);

  colorized_mesh_display::ColorizedMesh mesh_msg =
      colorized_mesh_display::fromPCLPolygonMesh(*input_mesh);

  while (ros::ok()) {
    colorized_mesh_display::ColorizedMeshStamped new_msg;
    new_msg.header.stamp = ros::Time::now();
    new_msg.header.frame_id = "world";
    new_msg.mesh = mesh_msg;
    mesh_pub.publish(new_msg);

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
