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
bool MesherMapper::CreatePublishers(const ros::NodeHandle& n) { return true; }

// Initialize callbacks
bool MesherMapper::RegisterCallbacks(const ros::NodeHandle& n) { return true; }

// To publish optimized mesh
bool MesherMapper::PublishOptimizedMesh(const pcl::PolygonMesh& mesh) {}

// Use octree compression for this  (add)
// Think of it as incremental mesh simplification
bool MesherMapper::AddDeformationGraph(const pcl::PolygonMesh& new_mesh) {}
}  // namespace mesher_mapper