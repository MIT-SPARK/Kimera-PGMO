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
bool MesherMapper::Initialize(const ros::NodeHandle& n) {}

// Load deformation parameters
bool MesherMapper::LoadParameters(const ros::NodeHandle& n) {}

// Initialize publishers
bool MesherMapper::CreatePublishers(const ros::NodeHandle& n) {}

// Initialize callbacks
bool MesherMapper::RegisterCallbacks(const ros::NodeHandle& n) {}

// To publish optimized mesh
bool MesherMapper::PublishOptimizedMesh(const pcl::PolygonMesh& mesh) {}

// Use octree compression for this  (add)
// Think of it as incremental mesh simplification
bool MesherMapper::AddDeformationGraph(const pcl::PolygonMesh& new_mesh) {}
}  // namespace mesher_mapper