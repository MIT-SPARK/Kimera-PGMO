/**
 * @file   mesher_mapper_node.cpp
 * @brief  Main load for mesher mapper
 * @author Yun Chang
 */
#include <ros/ros.h>

#include "kimera_pgmo/MesherMapper.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "kimera_pgmo");
  ros::NodeHandle n("~");

  kimera_pgmo::MesherMapper kimera_pgmo;
  if (!kimera_pgmo.Initialize(n)) {
    ROS_ERROR("Failed to initialize Mesher Mapper.");
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
