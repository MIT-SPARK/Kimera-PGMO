/**
 * @file   kimera_rmpgo_node.cpp
 * @brief  Main load for mesher mapper
 * @author Yun Chang
 */
#include <ros/ros.h>

#include "mesher_mapper/KimeraRmpgo.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "kimera_rmpgo");
  ros::NodeHandle n("~");

  mesher_mapper::KimeraRmpgo mesher_mapper;
  if (!mesher_mapper.Initialize(n)) {
    ROS_ERROR("Failed to initialize Mesher Mapper.");
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
