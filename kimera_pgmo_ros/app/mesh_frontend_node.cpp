#include <config_utilities/parsing/context.h>

#include <rclcpp/rclcpp.hpp>

#include "kimera_pgmo_ros/mesh_frontend.h"

int main(int argc, char* argv[]) {
  config::initContext(argc, argv);
  rclcpp::init(argc, argv);

  const rclcpp::NodeOptions options;
  auto node = std::make_shared<kimera_pgmo::MeshFrontend>(options);
  rclcpp::spin(node);

  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
