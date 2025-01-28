#include <config_utilities/parsing/ros.h>
#include <ros/ros.h>

#include "kimera_pgmo_ros/mesh_frontend.h"
#include "kimera_pgmo_ros/ros_log_sink.h"

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
