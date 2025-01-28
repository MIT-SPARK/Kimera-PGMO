/**
 * @file   mesh_frontend.h
 * @brief  MeshFrontend class: process incoming meshes
 * @author Yun Chang
 */
#pragma once

#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>
#include <ros/ros.h>

#include "kimera_pgmo/mesh_frontend_interface.h"

namespace kimera_pgmo {

class MeshFrontend : public MeshFrontendInterface {
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
  explicit MeshFrontend(const Config& config, const ros::NodeHandle& nh);

  virtual ~MeshFrontend() = default;

 protected:
  void handleMesh(const kimera_pgmo_msgs::KimeraPgmoMesh::ConstPtr& mesh);

  void publishFullMesh() const;

  void publishSimplifiedMesh() const;

  ros::NodeHandle nh_;
  ros::Subscriber sub_;
  ros::Publisher full_pub_;
  ros::Publisher simplified_pub_;
  //! publish the factors corresponding to the edges of the simplified mesh
  ros::Publisher mesh_graph_pub_;
};

void declare_config(MeshFrontend::Config& config);

}  // namespace kimera_pgmo
