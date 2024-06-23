/**
 * @file   mesh_frontend.cpp
 * @brief  VoxbloxMeshFrontend class: process incoming meshes and sample it for
 * the mesh parts of the deformation graph
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/voxblox_mesh_frontend.h"

#include <config_utilities/config.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <kimera_pgmo/utils/pcl_mesh_interface.h>
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <pose_graph_tools_msgs/PoseGraph.h>
#include <pose_graph_tools_ros/conversions.h>

#include <chrono>

#include "kimera_pgmo_ros/conversion/mesh_conversion.h"
#include "kimera_pgmo_ros/voxblox_msg_interface.h"

namespace kimera_pgmo {

using kimera_pgmo_msgs::KimeraPgmoMesh;
using pose_graph_tools_msgs::PoseGraph;

void declare_config(VoxbloxMeshFrontend::Config& config) {
  using namespace config;
  name("VoxbloxMeshFrontend::Config");
  base<MeshFrontendInterface::Config>(config);
  field(config.queue_size, "queue_size");
  field(config.frame_id, "frame_id");
}

VoxbloxMeshFrontend::VoxbloxMeshFrontend(const Config& config,
                                         const ros::NodeHandle& nh)
    : MeshFrontendInterface(config), config(config), nh_(nh) {
  full_pub_ = nh_.advertise<KimeraPgmoMesh>("full_mesh", 1, false);
  simplified_pub_ = nh_.advertise<KimeraPgmoMesh>("deformation_graph_mesh", 10, false);
  mesh_graph_pub_ = nh_.advertise<PoseGraph>("mesh_graph_incremental", 100, true);
  sub_ = nh_.subscribe<voxblox_msgs::Mesh>(
      "mesh_in", config.queue_size, &VoxbloxMeshFrontend::handleMesh, this);
  ROS_INFO("Initialized mesh_frontend.");
}

void VoxbloxMeshFrontend::handleMesh(const voxblox_msgs::Mesh::ConstPtr& msg) {
  if (!msg) {
    return;
  }
  const VoxbloxMsgInterface mesh_interface(*msg);
  update(mesh_interface, msg->header.stamp.toSec());

  // Publish edges and nodes if subscribed
  if (mesh_graph_pub_.getNumSubscribers() > 0) {
    auto msg = pose_graph_tools::toMsg(*last_mesh_graph_);
    mesh_graph_pub_.publish(msg);
  }

  publishFullMesh();
  publishSimplifiedMesh();
}

void VoxbloxMeshFrontend::publishFullMesh() const {
  if (full_pub_.getNumSubscribers() == 0) {
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

  full_pub_.publish(msg);
}

void VoxbloxMeshFrontend::publishSimplifiedMesh() const {
  if (simplified_pub_.getNumSubscribers() == 0) {
    return;
  }

  auto msg = conversions::toMsg(config.robot_id,
                                *graph_vertices_,
                                *graph_triangles_,
                                *graph_stamps_,
                                config.frame_id);

  simplified_pub_.publish(msg);
}

}  // namespace kimera_pgmo
