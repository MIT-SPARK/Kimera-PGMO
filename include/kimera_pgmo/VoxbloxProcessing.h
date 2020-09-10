/**
 * @file   VoxbloxProcessing.h
 * @brief  VoxbloxProcessing class: process incoming voxblox meshes
 * @author Yun Chang
 */
#pragma once

#include <ros/ros.h>
#include <map>
#include <queue>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/VoxbloxUtils.h"

namespace kimera_pgmo {

class VoxbloxProcessing {
  friend class VoxbloxProcessingTest;

 public:
  /*! \brief Constructor for VoxbloxProcessing class, which is in charge of
   * converting from Voxblox msg type to our mesh type while storing and
   * compressing the full unoptimized mesh
   */
  VoxbloxProcessing(size_t robot_id = 0);
  ~VoxbloxProcessing();

  /*! \brief Initializes callbacks and publishers, and also parse the parameters
   *  - n: ROS node handle.
   */
  bool initialize(const ros::NodeHandle& n);

 protected:
  /*! \brief Load the parameters required by this class through ROS
   *  - n: ROS node handle
   */
  bool loadParameters(const ros::NodeHandle& n);

  /*! \brief Creates the ROS publishers used
   *  - n: ROS node handle
   */
  bool createPublishers(const ros::NodeHandle& n);

  /*! \brief Starts the callbacks in this class
   *  - n: ROS node handle
   */
  bool registerCallbacks(const ros::NodeHandle& n);

  /*! \brief Main callback of this class: receives the updated incremental mesh
   * from Voxblox or Kimera-Semantics
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   */
  void voxbloxCallback(const voxblox_msgs::Mesh::ConstPtr& msg);

  // Creates partial mesh while updating the full mesh and also the last
  // detected mesh blocks
  /*! \brief Creates a partial mesh from the latest incremental mesh from the
   * callback and add the partial mesh to the full mesh and compress
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   */
  pcl::PolygonMesh processVoxbloxMesh(const voxblox_msgs::Mesh::ConstPtr& msg);

  /*! \brief Publish a mesh (used in callback to publish the partial mesh
   * created from the incremental mesh converted from latest msg from callback)
   *  - mesh: mesh to publish in pcl PolygonMesh format
   *  - stamp: timestamp
   */
  void publishPartialMesh(const pcl::PolygonMesh& mesh,
                          const ros::Time& stamp) const;

  /*! \brief Publish the full (compressed) mesh stored
   *  - stamp: timestamp
   */
  void publishFullMesh(const ros::Time& stamp) const;

  // Class arguments
  ros::Subscriber voxblox_sub_;
  ros::Publisher full_mesh_pub_;
  ros::Publisher partial_mesh_pub_;

  double time_horizon_;  // only merge meshes for the blocks detected
                         // within defined time horizon (secs)

  OctreeCompressionPtr compression_;  // Allow compression of full mesh

  size_t robot_id_;

  // Vertices of full mesh
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices_;
  // Triangles (connections) of full mesh
  std::vector<pcl::Vertices> triangles_;
};

}  // namespace kimera_pgmo
