/**
 * @file   mesh_frontend.cpp
 * @brief  MeshFrontend class: process incoming meshes and sample it for
 * the mesh parts of the deformation graph
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/mesh_frontend.h"

#include <config_utilities/config.h>
#include <config_utilities/parsing/context.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <kimera_pgmo/utils/pcl_mesh_interface.h>
#include <pose_graph_tools_ros/conversions.h>

#include <chrono>

#include "kimera_pgmo_ros/conversion/mesh.h"
#include "kimera_pgmo_ros/ros_log_sink.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::msg::Mesh;
using PoseGraphAdapter = pose_graph_tools::PoseGraphTypeAdapter;

void declare_config(MeshFrontend::Config& config) {
  using namespace config;
  name("MeshFrontend::Config");
  base<MeshFrontendInterface::Config>(config);
  field(config.queue_size, "queue_size");
  field(config.frame_id, "frame_id");
}

MeshFrontend::MeshFrontend(const rclcpp::NodeOptions& options)
    : MeshFrontendInterface(config::fromContext<Config>()),
      rclcpp::Node("mesh_frontend", options),
      config(config::fromContext<Config>()),
      mesh_graph_pub_(
          create_publisher<PoseGraphAdapter>("mesh_graph_incremental", 100)),
      full_pub_(create_publisher<Mesh>("full_mesh", 1)),
      simplified_pub_(create_publisher<Mesh>("deformation_graph_mesh", 10)),
      sub_(create_subscription<Mesh>(
          "mesh_in", config.queue_size, [this](const Mesh::ConstSharedPtr& msg) {
            handleMesh(msg);
          })) {
  logging::Logger::addSink("ros", std::make_shared<RosLogSink>(get_logger()));
  RCLCPP_INFO(get_logger(), "Initialized mesh_frontend.");
}

void MeshFrontend::handleMesh(const Mesh::ConstSharedPtr& msg) {
  const pcl::PolygonMesh mesh = conversions::fromMsg(*msg);
  const PclMeshInterface mesh_interface(mesh);
  update(mesh_interface, rclcpp::Time(msg->header.stamp).nanoseconds());

  // Publish edges and nodes if subscribed
  if (mesh_graph_pub_->get_subscription_count() > 0) {
    mesh_graph_pub_->publish(*last_mesh_graph_);
  }

  publishFullMesh();
  publishSimplifiedMesh();
}

void MeshFrontend::publishFullMesh() const {
  if (full_pub_->get_subscription_count() == 0) {
    return;
  }

  if (vertices_->size() == 0) {
    return;
  }

  auto msg = conversions::toMsg(config.robot_id,
                                *vertices_,
                                *triangles_,
                                *vertex_stamps_,
                                config.frame_id,
                                mesh_to_graph_idx_.get());
  full_pub_->publish(std::move(msg));
}

void MeshFrontend::publishSimplifiedMesh() const {
  if (simplified_pub_->get_subscription_count() == 0) {
    return;
  }

  auto msg = conversions::toMsg(config.robot_id,
                                *graph_vertices_,
                                *graph_triangles_,
                                *graph_stamps_,
                                config.frame_id);
  simplified_pub_->publish(std::move(msg));
}

}  // namespace kimera_pgmo
