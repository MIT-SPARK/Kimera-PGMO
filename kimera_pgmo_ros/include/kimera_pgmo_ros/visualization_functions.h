/**
 * @file   visualization_functions.h
 * @brief  Functions for visualizing structures
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <kimera_pgmo/deformation_graph.h>
#include <visualization_msgs/Marker.h>

namespace kimera_pgmo {

/*! \brief fill rviz markers for the deformation graph
 * - graph: deformation graph to make the message for
 * - stamp: ros timestamp of messages
 * - mesh_mesh_viz: marker for mesh-mesh edges
 * - pose_mesh_viz: marker for pose-mesh edges
 */
void fillDeformationGraphMarkers(const DeformationGraph& graph,
                                 const ros::Time& stamp,
                                 visualization_msgs::Marker& mesh_mesh_viz,
                                 visualization_msgs::Marker& pose_mesh_viz,
                                 const std::string& frame_id = "world");

}  // namespace kimera_pgmo
