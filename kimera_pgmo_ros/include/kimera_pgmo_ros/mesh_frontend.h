/**
 * @file   mesh_frontend.h
 * @brief  MeshFrontend class: process incoming meshes
 * @author Yun Chang
 */
#pragma once

#include <pose_graph_tools_ros/conversions.h>

#include <string>

#include <kimera_pgmo_msgs/msg/mesh.hpp>
#include <rclcpp/rclcpp.hpp>

#include "kimera_pgmo/mesh_frontend_interface.h"

namespace kimera_pgmo {

class MeshFrontend : public MeshFrontendInterface, public rclcpp::Node {
  friend class MeshFrontendTest;

 public:
  struct Config : MeshFrontendInterface::Config {
    int queue_size = 20;
    std::string frame_id = "world";
  } const config;

  /*! \brief Constructor for MeshFrontend class, which is in charge of
   * converting from mesh msg type to our mesh type while storing and
   * compressing the full unoptimized mesh
   */
  explicit MeshFrontend(const rclcpp::NodeOptions& options);

  virtual ~MeshFrontend() = default;

 protected:
  void handleMesh(const kimera_pgmo_msgs::msg::Mesh::ConstSharedPtr& mesh);

  void publishFullMesh() const;

  void publishSimplifiedMesh() const;

  //! publish the factors corresponding to the edges of the simplified mesh
  pose_graph_tools::PoseGraphPublisher mesh_graph_pub_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::Mesh>::SharedPtr full_pub_;
  rclcpp::Publisher<kimera_pgmo_msgs::msg::Mesh>::SharedPtr simplified_pub_;

  rclcpp::Subscription<kimera_pgmo_msgs::msg::Mesh>::SharedPtr sub_;
};

void declare_config(MeshFrontend::Config& config);

}  // namespace kimera_pgmo
