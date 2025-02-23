/**
 * @file   visualization_functions.h
 * @brief  Functions for visualizing structures
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <kimera_pgmo/deformation_graph.h>

#include <string>

#include <rclcpp/time.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace kimera_pgmo {

/*! \brief fill rviz markers for the deformation graph
 * - graph: deformation graph to make the message for
 * - stamp: ros timestamp of messages
 * - mesh_mesh_viz: marker for mesh-mesh edges
 * - pose_mesh_viz: marker for pose-mesh edges
 */
void fillDeformationGraphMarkers(const DeformationGraph& graph,
                                 const rclcpp::Time& stamp,
                                 visualization_msgs::msg::Marker& mesh_mesh_viz,
                                 visualization_msgs::msg::Marker& pose_mesh_viz,
                                 const std::string& frame_id = "world");

}  // namespace kimera_pgmo
