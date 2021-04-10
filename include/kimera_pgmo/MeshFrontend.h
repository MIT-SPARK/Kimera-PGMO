/**
 * @file   MeshFrontend.h
 * @brief  MeshFrontend class: process incoming voxblox meshes
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
#include <pose_graph_tools/PoseGraph.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/compression/OctreeCompression.h"
#include "kimera_pgmo/utils/CommonStructs.h"
#include "kimera_pgmo/utils/VoxbloxUtils.h"

namespace kimera_pgmo {

class MeshFrontend {
  friend class MeshFrontendTest;

 public:
  /*! \brief Constructor for MeshFrontend class, which is in charge of
   * converting from Voxblox msg type to our mesh type while storing and
   * compressing the full unoptimized mesh
   */
  MeshFrontend();
  ~MeshFrontend();

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

  /*! \brief Process the latest incremental mesh from the
   * callback and add the partial mesh to the full mesh and compress
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   */
  void processVoxbloxMesh(const voxblox_msgs::Mesh::ConstPtr& msg);

  /*! \brief Publish the full (compressed) mesh stored
   *  - stamp: timestamp
   */
  void publishFullMesh(const ros::Time& stamp) const;

  /*! \brief Publish the simplified mesh (used as the mesh part of deformation
   * graph)
   *  - stamp: timestamp
   */
  void publishSimplifiedMesh(const ros::Time& stamp) const;

  /*! \brief Publish the factors corresponding to the new edges added to the
   * simplified mesh / deformation graph and also the initial values (positions
   * of the new vertices added to the simplified mesh)
   *  - new_edges: new edges of type Edge (std::pair<Vertex, Vertex>)
   *  - new_indices: new vertices of type Vertex
   *  returns: published pose graph
   */
  pose_graph_tools::PoseGraph publishMeshGraph(
      const std::vector<Edge>& new_edges,
      const std::vector<size_t>& new_indices,
      const std_msgs::Header& header) const;

  /*! \brief Get last mesh graph created in voxblox callback for testing
   * purposes.
   */
  inline pose_graph_tools::PoseGraph getLastProcessedMeshGraph() const {
    return last_mesh_graph_;
  }

  /*! \brief Log the stats and the timing
   *  - filename: file to log to
   *  - callback_duration: callback time (mu-s)
   *  - num_indices: number of new indices to add to deformation graph
   *  - num_edges: number of new edges to add to deformation graph
   */
  void logTiming(const std::string& filename,
                 const int& callback_duration = 0,
                 const size_t& num_indices = 0,
                 const size_t& num_edges = 0) const;

  // Class arguments
  ros::Subscriber voxblox_sub_;
  ros::Publisher full_mesh_pub_;
  ros::Publisher simplified_mesh_pub_;
  ros::Publisher mesh_graph_pub_;  // publish the factors corresponding to the
                                   // edges of the simplified mesh

  double time_horizon_;  // only merge meshes for the blocks detected
                         // within defined time horizon (secs)

  OctreeCompressionPtr
      full_mesh_compression_;  // Allow compression of full mesh

  OctreeCompressionPtr d_graph_compression_;  // Compression to get simplified
                                              // mesh for deformation graph

  Graph simplified_mesh_graph_;  // Graph of simplified mesh (edges are the
                                 // factors in deformation graph)

  int robot_id_;

  bool initilized_log_;

  // Vertices of full mesh
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices_;
  // Triangles (connections) of full mesh
  std::vector<pcl::Vertices> triangles_;
  // Vertices of simplified mesh used for the deformation graph
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr graph_vertices_;
  // Triangles of the simplified mesh used for the deformation graph
  std::vector<pcl::Vertices> graph_triangles_;

  // Last pose graph msg created for testing purposes
  pose_graph_tools::PoseGraph last_mesh_graph_;

  // Save output
  std::string log_path_;
  // Log output to output_prefix_ folder
  bool log_output_;
};

}  // namespace kimera_pgmo
