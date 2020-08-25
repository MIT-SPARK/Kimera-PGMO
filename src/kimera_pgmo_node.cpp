/**
 * @file   kimera_pgmo_node.cpp
 * @brief  Main load for kimera pgmo
 * @author Yun Chang
 */
#include <ros/ros.h>

#include "kimera_pgmo/KimeraPgmo.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "kimera_pgmo");
  ros::NodeHandle n("~");

  kimera_pgmo::KimeraPgmo kimera_pgmo;
  if (!kimera_pgmo.initialize(n)) {
    ROS_ERROR("Failed to initialize Kimera Pgmo.");
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
