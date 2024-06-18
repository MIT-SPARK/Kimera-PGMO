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

#include "kimera_pgmo/MeshFrontend.h"

int main(int argc, char* argv[]) {
  // Initialize ROS node.
  ros::init(argc, argv, "mesh_frontend");
  ros::NodeHandle n("~");

  kimera_pgmo::MeshFrontend mesh_frontend;
  if (!mesh_frontend.initialize(n)) {
    ROS_ERROR("Failed to initialize Mesh Frontend node.");
    return EXIT_FAILURE;
  }

  ros::spin();

  return EXIT_SUCCESS;
}
