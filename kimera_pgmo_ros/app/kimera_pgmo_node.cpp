/**
 * @file   kimera_pgmo_node.cpp
 * @brief  Main load for kimera pgmo
 * @author Yun Chang
 */
#include <config_utilities/parsing/context.h>

#include <rclcpp/rclcpp.hpp>

#include "kimera_pgmo_ros/kimera_pgmo.h"

int main(int argc, char* argv[]) {
  config::initContext(argc, argv);
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;
  auto node = std::make_shared<kimera_pgmo::KimeraPgmo>(options);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
