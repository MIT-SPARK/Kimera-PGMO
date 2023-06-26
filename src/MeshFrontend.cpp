/**
 * @file   MeshFrontend.cpp
 * @brief  MeshFrontend class: process incoming voxblox meshes and sample it for
 * the mesh parts of the deformation graph
 * @author Yun Chang
 */
#include <chrono>

#include "kimera_pgmo/KimeraPgmoMesh.h"
#include "kimera_pgmo/MeshFrontend.h"
#include "kimera_pgmo/utils/CommonFunctions.h"

namespace kimera_pgmo {

bool loadFrontendConfigFromROS(const ros::NodeHandle& n, MeshFrontendConfig& config) {
  if (!n.getParam("horizon", config.time_horizon)) {
    return false;
  }
  if (!n.getParam("robot_id", config.robot_id)) {
    return false;
  }
  if (!n.getParam("output_mesh_resolution", config.mesh_resolution)) {
    return false;
  }
  if (!n.getParam("d_graph_resolution", config.d_graph_resolution)) {
    return false;
  }
  if (!n.getParam("full_compression_method", config.full_compression_method)) {
    return false;
  }
  if (!n.getParam("graph_compression_method", config.graph_compression_method)) {
    return false;
  }

  if (config.full_compression_method < 0 || config.full_compression_method > 2) {
    ROS_ERROR_STREAM("Invalid full compression mode "
                     << config.full_compression_method);
    return false;
  }
  if (config.graph_compression_method < 0 || config.graph_compression_method > 1) {
    ROS_ERROR_STREAM("Invalid dgraph compression mode "
                     << config.graph_compression_method);
    return false;
  }

  n.getParam("track_mesh_graph_mapping", config.b_track_mesh_graph_mapping);

  if (n.getParam("log_path", config.log_path)) {
    n.getParam("log_output", config.log_output);
    if (config.log_output) {
      ROS_INFO("Logging output to: %s", config.log_path.c_str());
    }
  }

  return true;
}

MeshFrontendPublisher::MeshFrontendPublisher(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  full_mesh_pub_ = nl.advertise<kimera_pgmo::KimeraPgmoMesh>("full_mesh", 1, false);
  simplified_mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("deformation_graph_mesh", 10, false);
  mesh_graph_pub_ =
      nl.advertise<pose_graph_tools::PoseGraph>("mesh_graph_incremental", 100, true);
}

void MeshFrontendPublisher::publishOutput(const MeshFrontendInterface& frontend,
                                          const std_msgs::Header& header) {
  // Publish edges and nodes if subscribed
  if (mesh_graph_pub_.getNumSubscribers() > 0) {
    mesh_graph_pub_.publish(frontend.last_mesh_graph_);
  }

  publishFullMesh(frontend);
  publishSimplifiedMesh(frontend, header.stamp);
}

void MeshFrontendPublisher::publishFullMesh(
    const MeshFrontendInterface& frontend) const {
  if (full_mesh_pub_.getNumSubscribers() == 0) return;
  if (frontend.getFullMeshVertices()->size() == 0) return;
  // convert to triangle mesh msg
  KimeraPgmoMesh mesh_msg =
      kimera_pgmo::PolygonMeshToPgmoMeshMsg(frontend.config_.robot_id,
                                            *frontend.getFullMeshVertices(),
                                            frontend.getFullMeshFaces(),
                                            frontend.getFullMeshTimes(),
                                            "world",
                                            frontend.getFullMeshToGraphMapping());
  // publish
  full_mesh_pub_.publish(mesh_msg);
  return;
}

void MeshFrontendPublisher::publishSimplifiedMesh(const MeshFrontendInterface& frontend,
                                                  const ros::Time& stamp) const {
  if (simplified_mesh_pub_.getNumSubscribers() == 0) return;
  // convert to triangle mesh msg
  mesh_msgs::TriangleMesh mesh_msg = kimera_pgmo::PolygonMeshToTriangleMeshMsg(
      *frontend.graph_vertices_, *frontend.graph_triangles_);

  // Create msg
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = stamp;
  new_msg.header.frame_id = "world";
  new_msg.mesh = mesh_msg;
  simplified_mesh_pub_.publish(new_msg);
  return;
}

MeshFrontend::MeshFrontend() : MeshFrontendInterface(), voxblox_queue_size_(20) {}

MeshFrontend::~MeshFrontend() {}

// Initialize parameters, publishers, and subscribers
bool MeshFrontend::initialize(const ros::NodeHandle& n) {
  n.getParam("voxblox_queue_size", voxblox_queue_size_);

  MeshFrontendConfig config;
  if (!loadFrontendConfigFromROS(n, config)) {
    ROS_ERROR("Failed to initialize mesh frontend!");
    return false;
  }
  MeshFrontendInterface::initialize(config);
  publisher_.reset(new MeshFrontendPublisher(n));
  addOutputCallback(std::bind(&MeshFrontendPublisher::publishOutput,
                              publisher_.get(),
                              std::placeholders::_1,
                              std::placeholders::_2));
  registerCallbacks(n);
  ROS_INFO("Initialized MeshFrontend.");
  return true;
}

bool MeshFrontend::registerCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  voxblox_sub_ = nl.subscribe("voxblox_mesh",
                              voxblox_queue_size_,
                              &MeshFrontendInterface::voxbloxCallback,
                              static_cast<MeshFrontendInterface*>(this));
  return true;
}

}  // namespace kimera_pgmo
