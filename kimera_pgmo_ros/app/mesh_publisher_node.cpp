/**
 * @file   kimera_pgmo_node.cpp
 * @brief  Main load for kimera pgmo
 * @author Yun Chang
 */

#include <config_utilities/config_utilities.h>
#include <config_utilities/parsing/ros.h>
#include <config_utilities/types/path.h>
#include <kimera_pgmo/utils/mesh_io.h>
#include <ros/assert.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>

#include <filesystem>

#include "kimera_pgmo_ros/conversion/mesh_conversion.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::KimeraPgmoMesh;

class MeshPublisherNode {
 public:
  struct Config {
    size_t robot_id = 0;
    std::string mesh_frame = "map";
    std::filesystem::path mesh_filepath;
  } const config;

  explicit MeshPublisherNode(const ros::NodeHandle& nh);
  ~MeshPublisherNode() = default;

 private:
  bool reload(std_srvs::Empty::Request&, std_srvs::Empty::Response&);
  void publishMesh();

  ros::NodeHandle nh_;
  ros::Publisher pub_;
  ros::ServiceServer reload_service_;
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

MeshPublisherNode::MeshPublisherNode(const ros::NodeHandle& nh)
    : config(config::checkValid(config::fromRos<Config>(nh))), nh_(nh) {
  ROS_INFO_STREAM("Starting publisher node with\n" << config::toString(config));
  pub_ = nh_.advertise<KimeraPgmoMesh>("mesh", 1, true);
  publishMesh();

  reload_service_ = nh_.advertiseService("reload", &MeshPublisherNode::reload, this);
}

bool MeshPublisherNode::reload(std_srvs::Empty::Request&, std_srvs::Empty::Response&) {
  publishMesh();
  return true;
}

void MeshPublisherNode::publishMesh() {
  ROS_INFO_STREAM("Loading mesh from: " << config.mesh_filepath);
  pcl::PolygonMesh mesh;
  std::vector<Timestamp> stamps;
  ReadMeshWithStampsFromPly(config.mesh_filepath, mesh, &stamps);
  const auto num_vertices = mesh.cloud.height * mesh.cloud.width;
  ROS_INFO_STREAM("Loaded mesh with " << num_vertices << " vertices, "
                                      << mesh.polygons.size() << " faces, and "
                                      << stamps.size() << " timestamps");

  auto msg = conversions::toMsg(config.robot_id, mesh, stamps, config.mesh_frame);
  ROS_ASSERT_MSG(msg != nullptr, "valid mesh required");
  msg->header.stamp = ros::Time::now();
  pub_.publish(msg);
}

}  // namespace kimera_pgmo

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "mesh_publisher_node");
  ros::NodeHandle nh("~");

  kimera_pgmo::MeshPublisherNode node(nh);
  ros::spin();
  return EXIT_SUCCESS;
}
