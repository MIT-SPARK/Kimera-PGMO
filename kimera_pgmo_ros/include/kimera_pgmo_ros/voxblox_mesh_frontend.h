/**
 * @file   voxblox_mesh_frontend.h
 * @brief  VoxbloxMeshFrontend class: process incoming voxblox mesh messages
 * @author Yun Chang
 */
#pragma once

#include <ros/ros.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/mesh_frontend_interface.h"

namespace kimera_pgmo {

class VoxbloxMeshFrontend : public MeshFrontendInterface {
  friend class VoxbloxMeshFrontendTest;

 public:
  struct Config : MeshFrontendInterface::Config {
    int queue_size = 20;
    std::string frame_id = "world";
  } const config;

  /*! \brief Constructor for VoxbloxMeshFrontend class, which is in charge of
   * converting from Voxblox msg type to our mesh type while storing and
   * compressing the full unoptimized mesh
   */
  explicit VoxbloxMeshFrontend(const Config& config, const ros::NodeHandle& nh);

  virtual ~VoxbloxMeshFrontend() = default;

 protected:
  void handleMesh(const voxblox_msgs::Mesh::ConstPtr& mesh);

  void publishFullMesh() const;

  void publishSimplifiedMesh() const;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher full_pub_;
  ros::Publisher simplified_pub_;
  //! publish the factors corresponding to the edges of the simplified mesh
  ros::Publisher mesh_graph_pub_;
};

void declare_config(VoxbloxMeshFrontend::Config& config);

}  // namespace kimera_pgmo
