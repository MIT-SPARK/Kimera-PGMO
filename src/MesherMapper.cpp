/**
 * @file   MesherMapper.cpp
 * @brief  MesherMapper class: Main class and ROS interface
 * @author Yun Chang
 */
#include <geometry_msgs/PoseStamped.h>
#include <cmath>

#include "mesher_mapper/MesherMapper.h"

namespace mesher_mapper {

// Constructor
MesherMapper::MesherMapper() : save_optimized_mesh_(false) {}
MesherMapper::~MesherMapper() {}

// Initialize parameters, publishers, and subscribers and deformation graph
bool MesherMapper::Initialize(const ros::NodeHandle& n) {
  // start the two mesh compression modules: one for deformation graph and one
  // for the overall map mesh
  double deformation_graph_resolution;
  if (!n.getParam("d_graph_resolution", deformation_graph_resolution))
    return false;

  if (!d_graph_compression_.Initialize(
          n, deformation_graph_resolution, "deformation")) {
    ROS_ERROR(
        "MesherMapper: Failed to intialize deformation graph compression "
        "module.");
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
  if (!n.getParam("embed_trajectory/max_delta_t", embed_delta_t_)) return false;
  if (!n.getParam("embed_trajectory/max_delta_r", embed_delta_r_)) return false;
  if (n.getParam("output_ply_file", output_file_)) {
    save_optimized_mesh_ = true;
    ROS_INFO("Saving optimized mesh to: %s", output_file_.c_str());
  }
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
  deform_input_sub_ = nl.subscribe(
      "distortion_pose", 10, &MesherMapper::LoopClosureCallback, this);

  trajectory_sub_ =
      nl.subscribe("trajectory", 10, &MesherMapper::TrajectoryCallback, this);

  input_mesh_sub_ =
      nl.subscribe("input_mesh", 1, &MesherMapper::MeshCallback, this);

  // start timer
  update_timer_ =
      nl.createTimer(1.0, &MesherMapper::ProcessTimerCallback, this);
  return true;
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

void MesherMapper::TrajectoryCallback(const nav_msgs::Path::ConstPtr& msg) {
  // Convert trajectory to gtsam
  std::vector<gtsam::Pose3> current_trajectory;
  for (geometry_msgs::PoseStamped ros_pose : msg->poses) {
    gtsam::Pose3 pose = RosToGtsam(ros_pose.pose);
    current_trajectory.push_back(pose);
  }

  // Add to current trajectory and also embed into deformation graph
  // Get the latest observation timestamp of the vertices
  double msg_time = msg->header.stamp.toSec();
  std::vector<double> latest_observed_times;
  d_graph_compression_.getVerticesTimestamps(&latest_observed_times);
  // Get the vertices of the simplified mesh
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr simplified_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  d_graph_compression_.getVertices(simplified_vertices);
  size_t trajectory_len = trajectory_.size();
  for (size_t i = 0; i < current_trajectory.size(); i++) {
    Vertices valences;
    gtsam::Point3 pos = current_trajectory[i].translation();
    // connect to nodes that are proximate in spacetime
    for (size_t j = 0; j < latest_observed_times.size(); j++) {
      if (abs(msg_time - latest_observed_times[j]) < embed_delta_t_) {
        pcl::PointXYZRGBA candidate_pos = simplified_vertices->points[j];
        double dist = std::sqrt(
            (pos.x() - candidate_pos.x) * (pos.x() - candidate_pos.x) +
            (pos.y() - candidate_pos.y) * (pos.y() - candidate_pos.y) +
            (pos.z() - candidate_pos.z) * (pos.z() - candidate_pos.z));
        if (dist < embed_delta_r_) {
          valences.push_back(j);
        }
      }
    }
    ROS_INFO_STREAM("Trajectory node " << i << " connected to "
                                       << valences.size() << " points out of "
                                       << latest_observed_times.size()
                                       << " in simplified mesh");
    if (i >= trajectory_len) {
      trajectory_.push_back(current_trajectory[i]);
      // add node to deform graph
      deformation_graph_.addNode(
          pcl::PointXYZ(pos.x(), pos.y(), pos.z()), valences, true);
    } else {
      deformation_graph_.updateNodeValence(i, valences, true);
    }
  }

  // Add distortions
  ROS_ERROR("SINCE CLEAR MEASUREMENTS NO LONGER WORK THIS IS DEPRECATED.");
  // deformation_graph_.clearMeasurements();  // but first clear previous input
  for (size_t i = 0; i < trajectory_.size(); i++) {
    gtsam::Pose3 distortion = trajectory_[i].between(current_trajectory[i]);
    deformation_graph_.addNodeMeasurement(i, distortion);
    // i.e. if no loop closures, distortion should be identity
  }

  pcl::PolygonMesh simplified_mesh;
  // Fill in simplified mesh
  std::vector<pcl::Vertices> simplified_polygons;
  d_graph_compression_.getPolygons(&simplified_polygons);
  simplified_mesh.polygons = simplified_polygons;
  pcl::toPCLPointCloud2(*simplified_vertices, simplified_mesh.cloud);

  // Check if new portions added for deformation graph
  deformation_graph_.updateMesh(simplified_mesh);
  deformation_graph_.update();
  Graph graph = deformation_graph_.getGraph();
  deformation_graph_.optimize();
  ROS_INFO("MesherMapper: Optimized with trajectory of length %d.",
           trajectory_.size());
}

void MesherMapper::LoopClosureCallback(
    const mesher_mapper::AbsolutePoseStamped::ConstPtr& msg) {
  // Enforce the set points
  // Get new simplified mesh from compressor
  pcl::PolygonMesh simplified_mesh;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr simplified_vertices(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  d_graph_compression_.getVertices(simplified_vertices);
  std::vector<pcl::Vertices> simplified_polygons;
  d_graph_compression_.getPolygons(&simplified_polygons);
  simplified_mesh.polygons = simplified_polygons;
  pcl::toPCLPointCloud2(*simplified_vertices, simplified_mesh.cloud);

  // Check if new portions added for deformation graph
  if (deformation_graph_.getNumVertices() <
      simplified_vertices->points.size()) {
    deformation_graph_ = DeformationGraph();
    deformation_graph_.updateMesh(simplified_mesh);
    deformation_graph_.update();
  }

  // add relative measurement
  deformation_graph_.addMeasurement(Vertex(msg->idx), msg->pose);
  ROS_INFO("MesherMapper: Added 1 distortion transform.");
  // optimize with new relative mesurement
  deformation_graph_.optimize();
}

void MesherMapper::MeshCallback(
    const mesh_msgs::TriangleMeshStamped::ConstPtr& mesh_msg) {
  input_mesh_ = TriangleMeshMsgToPolygonMesh(mesh_msg->mesh);
  return;
}

void MesherMapper::ProcessTimerCallback(const ros::TimerEvent& ev) {
  // Update optimized mesh
  optimized_mesh_ = deformation_graph_.deformMesh(input_mesh_);
  PublishOptimizedMesh();

  // Save mesh
  if (save_optimized_mesh_) {
    WriteMeshToPly(output_file_, optimized_mesh_);
  }
}

}  // namespace mesher_mapper