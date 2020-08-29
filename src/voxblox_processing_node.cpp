/**
 * @file   voxblox_to_mesh.cpp
 * @brief  node to convert voxblox msg to mesh msg (last mesh block)
 * @author Yun Chang
 */

#include <ros/ros.h>
#include <map>

#include <mesh_msgs/TriangleMeshStamped.h>
#include <pcl/PolygonMesh.h>
#include <voxblox_msgs/Mesh.h>

#include "kimera_pgmo/VoxbloxProcessing.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "voxblox_processing");
  ros::NodeHandle n("~");

  kimera_pgmo::VoxbloxProcessing voxblox_processing;
  if (!voxblox_processing.initialize(n)) {
    ROS_ERROR("Failed to initialize Voxblox Processing node.");
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
