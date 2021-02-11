/**
 * @file   kimera_pgmo_multi_node.cpp
 * @brief  Multi robot basestation kimera pgmo
 * @author Yun Chang
 */
#include <ros/ros.h>

#include "kimera_pgmo/KimeraPgmoMulti.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "kimera_pgmo");
  ros::NodeHandle n("~");

  std::vector<size_t> robots(3);
  std::iota(
      std::begin(robots), std::end(robots), 0);  // Fill with 0, 1, ..., 99.

  kimera_pgmo::KimeraPgmoMulti kimera_pgmo(robots);
  if (!kimera_pgmo.initialize(n)) {
    ROS_ERROR("Failed to initialize Kimera Pgmo Multi.");
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
