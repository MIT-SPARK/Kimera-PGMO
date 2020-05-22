/**
 * @file   mesher_mapper_node.cpp
 * @brief  Main load for mesher mapper
 * @author Yun Chang
 */
#include <ros/ros.h>

#include "mesher_mapper/MesherMapper.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "mesher_mapper");
  ros::NodeHandle n("~");

  mesher_mapper::MesherMapper mesher_mapper;
  if (!mesher_mapper.Initialize(n)) {
    ROS_ERROR("Failed to initialize Mesher Mapper.");
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
