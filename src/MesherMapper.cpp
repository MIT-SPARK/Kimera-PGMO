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
bool MesherMapper::LoadParameters(const ros::NodeHandle& n) {
  if (!n.getParam("frame_id", frame_id_)) return false;
  return true;
}

// Initialize publishers
bool MesherMapper::CreatePublishers(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  optimized_mesh_pub_ =
      nl.advertise<mesh_msgs::TriangleMeshStamped>("optimized_mesh", 1, true);
  return true;
}

// Initialize callbacks
bool MesherMapper::RegisterCallbacks(const ros::NodeHandle& n) {
  ros::NodeHandle nl(n);
  input_mesh_sub_ =
      nl.subscribe("input_mesh", 10, &MesherMapper::MeshCallback, this);
  deform_input_sub_ = nl.subscribe(
      "distortion_pose", 10, &MesherMapper::LoopClosureCallback, this);

  // start timer
  update_timer_ =
      nl.createTimer(1.0, &MesherMapper::ProcessTimerCallback, this);
  return true;
}

void MesherMapper::MeshCallback(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  input_mesh_ = TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
}

// To publish optimized mesh
bool MesherMapper::PublishOptimizedMesh() {
  mesh_msgs::TriangleMesh mesh_msg =
      PolygonMeshToTriangleMeshMsg(optimized_mesh_);

  // Create msg
  mesh_msgs::TriangleMeshStamped new_msg;
  new_msg.header.stamp = ros::Time::now();
  new_msg.header.frame_id = frame_id_;
  new_msg.mesh = mesh_msg;

  optimized_mesh_pub_.publish(new_msg);
  return true;
}

void MesherMapper::LoopClosureCallback(
    const mesher_mapper::AbsolutePoseStamped::ConstPtr& msg) {
  // Enforce the set points
  // Get new simplified mesh from compressor
  pcl::PolygonMesh simplified_mesh;
  pcl::PointCloud<pcl::PointXYZ>::Ptr simplified_vertices(
      new pcl::PointCloud<pcl::PointXYZ>);
  compression_.getVertices(simplified_vertices);
  std::vector<pcl::Vertices> simplified_polygons;
  compression_.getPolygons(&simplified_polygons);
  simplified_mesh.polygons = simplified_polygons;
  pcl::toPCLPointCloud2(*simplified_vertices, simplified_mesh.cloud);

  // Check if new portions added for deformation graph
  if (deformation_graph_.getNumVertices() <
      simplified_vertices->points.size()) {
    deformation_graph_ = DeformationGraph();
    deformation_graph_.createFromMesh(simplified_mesh);
  }

  // add relative measurement
  deformation_graph_.addMeasurement(Vertex(msg->idx), msg->pose);
  ROS_INFO("MesherMapper: Added 1 distortion transform.");
  // optimize with new relative mesurement
  deformation_graph_.optimize();
}

void MesherMapper::ProcessTimerCallback(const ros::TimerEvent& ev) {
  // Update optimized mesh
  optimized_mesh_ = deformation_graph_.deformMesh(input_mesh_);
  PublishOptimizedMesh();
}

}  // namespace mesher_mapper