/**
 * @file   voxblox_to_mesh.cpp
 * @brief  node to convert voxblox msg to mesh msg (last mesh block)
 * @author Yun Chang
 */

#include <config_utilities/parsing/ros.h>
#include <ros/ros.h>

#include "kimera_pgmo_ros/MeshFrontend.h"
#include "kimera_pgmo_ros/RosLogSink.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "mesh_frontend");
  ros::NodeHandle n("~");

  logging::Logger::addSink("ros", std::make_shared<kimera_pgmo::RosLogSink>());

  const auto pgmo_config = config::fromRos<kimera_pgmo::MeshFrontend::Config>(n);
  kimera_pgmo::MeshFrontend mesh_frontend(pgmo_config, n);
  ros::spin();

  return EXIT_SUCCESS;
}
