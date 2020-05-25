/**
 * @file   MesherMapper.cpp
 * @brief  MesherMapper class: Main class and ROS interface
 * @author Yun Chang
 */

#include "mesher_mapper/MesherMapper.h"

namespace mesher_mapper {

// Constructor
MesherMapper::MesherMapper() {}
MesherMapper::~MesherMapper() {}

// Initialize parameters, publishers, and subscribers and deformation graph
bool MesherMapper::Initialize(const ros::NodeHandle& n) {
  if (!compression_.Initialize(n)) {
    ROS_ERROR("MesherMapper: Failed to intialize mesh compression module.");
  }

  if (!LoadParameters(n)) {
    ROS_ERROR("MesherMapper: Failed to load parameters.");
  }

  if (!CreatePublishers(n)) {
    ROS_ERROR("MesherMapper: Failed to create publishers.");
  }

  if (!RegisterCallbacks(n)) {
    ROS_ERROR("MesherMapper: Failed to register callbacks.");
  }

  ROS_INFO("Initializes Mesher Mapper.");

  return true;
}

// Load deformation parameters
bool MesherMapper::LoadParameters(const ros::NodeHandle& n) { return true; }

// Initialize publishers
bool MesherMapper::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  optimized_mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("optimized_mesh", 10, true);
  return true;
}

// Initialize callbacks
bool MesherMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  input_mesh_sub_ =
      nl.subscribe("input_mesh", 10, &MesherMapper::MeshCallback, this);
  return true;
}

void MesherMapper::MeshCallback(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  pcl::PolygonMesh polygon_mesh = TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
  // Get new simplified mesh from compressor
  pcl::PointCloud<pcl::PointXYZ>::Ptr simplified_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  compression_.getVertices(simplified_vertices);
  std::vector<pcl::Vertices> simplified_polygons;
  compression_.getPolygons(&simplified_polygons);

  // Check if new portions added
  if (deformation_graph_.getNumVertices() <
      simplified_vertices->points.size()) {
    deformation_graph_ = DeformationGraph();
    deformation_graph_.createFromMesh(polygon_mesh);
  }

  // TODO Actually should move this to callback for user input?
}

// To publish optimized mesh
bool MesherMapper::PublishOptimizedMesh(const pcl::PolygonMesh& mesh) {}

// Use octree compression for this  (add)
// Think of it as incremental mesh simplification
bool MesherMapper::AddDeformationGraph(const pcl::PolygonMesh& new_mesh) {}
}  // namespace mesher_mapper