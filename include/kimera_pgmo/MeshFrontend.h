/**
 * @file   MeshFrontend.h
 * @brief  MeshFrontend class: process incoming voxblox meshes
 * @author Yun Chang
 */
#pragma once

#include <ros/ros.h>
#include <map>
#include <queue>
#include <unordered_map>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pose_graph_tools/PoseGraph.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/compression/MeshCompression.h"
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
  bool initialize(const ros::NodeHandle& n, bool should_register_callbacks = true);

  /*! /brief Get curent frontend vertices
   *  /returns Current vertex pointcloud
   */
  inline pcl::PointCloud<pcl::PointXYZRGBA>::Ptr getFullMeshVertices() const {
    return vertices_;
  }

  /*! /brief Get active indices
   *  /returns Active indices (i.e. inside frontend time-horizon) of the full
   * mesh
   */
  inline const std::vector<size_t>& getActiveFullMeshVertices() const {
    return full_mesh_compression_->getActiveVerticesIndex();
  }

  /*! /brief Check if the frontend has received input from voxblox since the
   * last time this function was called
   */
  inline bool wasFrontendUpdated(bool clear_update_flag = true) {
    bool prev_flag = voxblox_update_called_;
    voxblox_update_called_ = clear_update_flag ? false : voxblox_update_called_;
    return prev_flag;
  }

  /*! \brief Get the time horizion (in seconds) of the mesh compression
   */
  inline double getMeshTimeHorizon() const { return time_horizon_; }

  /*! \brief Get the last message's timestamps
   */
  inline ros::Time getLastMsgTime() const { return last_mesh_msg_time_; }

  /*! \brief Get the mappings from vxblx msg to graph index for tracking.
   */
  inline const VoxbloxIndexMapping& getVoxbloxMsgMapping() const {
    return *vxblx_msg_to_graph_idx_;
  }

  /*! \brief Main callback of this class: receives the updated incremental mesh
   * from Voxblox or Kimera-Semantics
   *  - msg: mesh msg from Voxblox or Kimera Semantics
   */
  void voxbloxCallback(const voxblox_msgs::Mesh::ConstPtr& msg);

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

  MeshCompressionPtr full_mesh_compression_;  // Allow compression of full mesh

  MeshCompressionPtr d_graph_compression_;  // Compression to get simplified
                                            // mesh for deformation graph

  Graph simplified_mesh_graph_;  // Graph of simplified mesh (edges are the
                                 // factors in deformation graph)

  int robot_id_;

  bool initilized_log_;

  ros::Time last_mesh_msg_time_;  // last message processed timestamp

  // Vertices of full mesh
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices_;
  // Triangles (connections) of full mesh
  boost::shared_ptr<std::vector<pcl::Vertices>> triangles_;
  // Vertices of simplified mesh used for the deformation graph
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr graph_vertices_;
  // Triangles of the simplified mesh used for the deformation graph
  boost::shared_ptr<std::vector<pcl::Vertices>> graph_triangles_;

  // Last pose graph msg created for testing purposes
  pose_graph_tools::PoseGraph last_mesh_graph_;

  // Book keeping for indices
  boost::shared_ptr<VoxbloxIndexMapping> vxblx_msg_to_graph_idx_;

  // Save output
  std::string log_path_;
  // Log output to output_prefix_ folder
  bool log_output_;

  // queue size for voxblox
  int voxblox_queue_size_;

  // whether or not voxbloxCallback triggered since wasFrontendUpdated was
  // called
  bool voxblox_update_called_;
};
}  // namespace kimera_pgmo
