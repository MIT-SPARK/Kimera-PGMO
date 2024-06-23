#include <config_utilities/parsing/ros.h>
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <ros/ros.h>

#include "kimera_pgmo_ros/ros_log_sink.h"
#include "kimera_pgmo_ros/voxblox_mesh_frontend.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "mesh_frontend");
  ros::NodeHandle n("~");

  logging::Logger::addSink("ros", std::make_shared<kimera_pgmo::RosLogSink>());

  const auto pgmo_config = config::fromRos<kimera_pgmo::VoxbloxMeshFrontend::Config>(n);
  kimera_pgmo::VoxbloxMeshFrontend mesh_frontend(pgmo_config, n);
  ros::spin();

  return EXIT_SUCCESS;
}
