/**
 * @file   MeshFrontend.h
 * @brief  MeshFrontend class: process incoming voxblox meshes
 * @author Yun Chang
 */
#pragma once

#include <ros/ros.h>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pose_graph_tools/PoseGraph.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/MeshFrontendInterface.h"

namespace kimera_pgmo {

bool loadFrontendConfigFromROS(const ros::NodeHandle& n, MeshFrontendConfig& config);

class MeshFrontendPublisher {
 public:
  /*! \brief Constructor for MeshFrontend class, which is in charge of
   * converting from Voxblox msg type to our mesh type while storing and
   * compressing the full unoptimized mesh
   */
  MeshFrontendPublisher(const ros::NodeHandle& nh);

  virtual ~MeshFrontendPublisher() = default;

  /*! \brief Publish frontend info to ROS
   *  - frontend: base class const reference (unused)
   *  - header: voxblox msg header
   */
  void publishOutput(const MeshFrontendInterface& frontend,
                     const std_msgs::Header& header);

 protected:
  /*! \brief Publish the full (compressed) mesh stored
   *  - stamp: timestamp
   */
  void publishFullMesh(const MeshFrontendInterface& frontend) const;

  /*! \brief Publish the simplified mesh (used as the mesh part of deformation
   * graph)
   *  - stamp: timestamp
   */
  void publishSimplifiedMesh(const MeshFrontendInterface& frontend,
                             const ros::Time& stamp) const;

  ros::Publisher full_mesh_pub_;
  ros::Publisher simplified_mesh_pub_;
  ros::Publisher mesh_graph_pub_;  // publish the factors corresponding to the
                                   // edges of the simplified mesh
};

class MeshFrontend : public MeshFrontendInterface {
  friend class MeshFrontendTest;

 public:
  /*! \brief Constructor for MeshFrontend class, which is in charge of
   * converting from Voxblox msg type to our mesh type while storing and
   * compressing the full unoptimized mesh
   */
  MeshFrontend();

  virtual ~MeshFrontend();

  /*! \brief Initializes callbacks and publishers, and also parse the parameters
   *  - n: ROS node handle.
   */
  bool initialize(const ros::NodeHandle& n);

 protected:
  /*! \brief Starts the callbacks in this class
   *  - n: ROS node handle
   */
  bool registerCallbacks(const ros::NodeHandle& n);

  // Class arguments
  ros::Subscriber voxblox_sub_;
  std::unique_ptr<MeshFrontendPublisher> publisher_;
  int voxblox_queue_size_;  // queue size for voxblox
};

}  // namespace kimera_pgmo
