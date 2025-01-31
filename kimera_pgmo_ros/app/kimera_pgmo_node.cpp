/**
 * @file   kimera_pgmo_node.cpp
 * @brief  Main load for kimera pgmo
 * @author Yun Chang
 */
#include <ros/ros.h>

#include "kimera_pgmo_ros/kimera_pgmo.h"
#include "kimera_pgmo_ros/ros_log_sink.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "kimera_pgmo");
  ros::NodeHandle n("~");

  logging::Logger::addSink("ros", std::make_shared<kimera_pgmo::RosLogSink>());

  kimera_pgmo::KimeraPgmo kimera_pgmo(n);

  ros::spin();

  return EXIT_SUCCESS;
}
