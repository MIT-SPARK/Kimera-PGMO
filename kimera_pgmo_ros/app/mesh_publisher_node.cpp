/**
 * @file   kimera_pgmo_node.cpp
 * @brief  Main load for kimera pgmo
 * @author Yun Chang
 */

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/commandline.h>
#include <config_utilities/types/path.h>
#include <kimera_pgmo/utils/mesh_io.h>

#include <filesystem>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/empty.hpp>

#include "kimera_pgmo_ros/conversion/mesh.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::msg::Mesh;

class MeshPublisherNode : public rclcpp::Node {
 public:
  struct Config {
    size_t robot_id = 0;
    std::string mesh_frame = "map";
    std::filesystem::path mesh_filepath;
  } const config;

  explicit MeshPublisherNode(const Config& config);
  ~MeshPublisherNode() = default;

 private:
  void reload(const std_srvs::srv::Empty::Request::SharedPtr&,
              std_srvs::srv::Empty::Response::SharedPtr);
  void publishMesh();

  rclcpp::Publisher<Mesh>::SharedPtr pub_;
  rclcpp::Service<std_srvs::srv::Empty>::SharedPtr reload_service_;
};

void declare_config(MeshPublisherNode::Config& config) {
  using namespace config;
  name("MeshPublisherNode::Config");
  field(config.robot_id, "robot_id");
  field(config.mesh_frame, "mesh_frame");
  field<Path>(config.mesh_filepath, "mesh_filepath");
  checkCondition(!config.mesh_frame.empty(), "mesh_frame");
  check<Path::Exists>(config.mesh_filepath, "mesh_filepath");
}

MeshPublisherNode::MeshPublisherNode(const Config& _config)
    : rclcpp::Node("mesh_publisher_node"), config(config::checkValid(_config)) {
  using namespace std::placeholders;
  RCLCPP_INFO_STREAM(get_logger(),
                     "Starting publisher node with\n"
                         << config::toString(config));
  pub_ = create_publisher<Mesh>("mesh", rclcpp::QoS(1).transient_local());
  publishMesh();

  reload_service_ = create_service<std_srvs::srv::Empty>(
      "reload", std::bind(&MeshPublisherNode::reload, this, _1, _2));
}

void MeshPublisherNode::reload(const std_srvs::srv::Empty::Request::SharedPtr&,
                               std_srvs::srv::Empty::Response::SharedPtr) {
  publishMesh();
}

void MeshPublisherNode::publishMesh() {
  RCLCPP_INFO_STREAM(get_logger(), "Loading mesh from: " << config.mesh_filepath);
  pcl::PolygonMesh mesh;
  std::vector<Timestamp> stamps;
  ReadMeshWithStampsFromPly(config.mesh_filepath, mesh, &stamps);
  const auto num_vertices = mesh.cloud.height * mesh.cloud.width;
  RCLCPP_INFO_STREAM(get_logger(),
                     "Loaded mesh with " << num_vertices << " vertices, "
                                         << mesh.polygons.size() << " faces, and "
                                         << stamps.size() << " timestamps");

  auto msg = conversions::toMsg(config.robot_id, mesh, stamps, config.mesh_frame);
  if (!msg) {
    RCLCPP_ERROR(get_logger(), "Unable to convert mesh!");
    return;
  }

  msg->header.stamp = get_clock()->now();
  pub_->publish(std::move(msg));
}

}  // namespace kimera_pgmo

int main(int argc, char* argv[]) {
  const auto config =
      config::fromCLI<kimera_pgmo::MeshPublisherNode::Config>(argc, argv);

  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<kimera_pgmo::MeshPublisherNode>(config));
  rclcpp::shutdown();
  return EXIT_SUCCESS;
}
