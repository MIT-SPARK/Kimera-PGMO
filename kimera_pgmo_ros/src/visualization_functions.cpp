#include "kimera_pgmo_ros/visualization_functions.h"

#include <tf2_eigen/tf2_eigen.h>

namespace kimera_pgmo {

void fillDeformationGraphMarkers(const DeformationGraph& graph,
                                 const ros::Time& stamp,
                                 visualization_msgs::Marker& mesh_mesh_viz,
                                 visualization_msgs::Marker& pose_mesh_viz,
                                 const std::string& frame_id) {
  // First get the latest estimates and factors
  const auto& graph_values = graph.getGtsamValues();
  const auto& graph_factors = graph.getGtsamFactors();

  // header for the mesh to mesh edges
  mesh_mesh_viz.header.frame_id = frame_id;
  mesh_mesh_viz.header.stamp = stamp;
  mesh_mesh_viz.id = 0;
  mesh_mesh_viz.action = visualization_msgs::Marker::ADD;
  mesh_mesh_viz.type = visualization_msgs::Marker::LINE_LIST;
  mesh_mesh_viz.scale.x = 0.02;

  // header for the pose to mesh edges
  pose_mesh_viz.header.frame_id = frame_id;
  pose_mesh_viz.header.stamp = stamp;
  pose_mesh_viz.id = 0;
  pose_mesh_viz.action = visualization_msgs::Marker::ADD;
  pose_mesh_viz.type = visualization_msgs::Marker::LINE_LIST;
  pose_mesh_viz.scale.x = 0.02;

  for (auto factor : graph_factors) {
    // Only interested in edges here
    if (factor->keys().size() != 2) {
      continue;
    }

    const gtsam::Symbol& front = factor->front();
    const gtsam::Symbol& back = factor->back();

    const bool front_is_pose_vertex =
        (robot_prefix_to_id.find(front.chr()) != robot_prefix_to_id.end());
    const bool back_is_pose_vertex =
        (robot_prefix_to_id.find(back.chr()) != robot_prefix_to_id.end());

    // Three types: pose-to-pose, pose-to-mesh, mesh-to-mesh
    // color accordingly
    std_msgs::ColorRGBA color;
    if (front_is_pose_vertex && back_is_pose_vertex) {
      // pose-to-pose
      continue;
    }

    if (!front_is_pose_vertex && !back_is_pose_vertex) {
      // mesh-to-mesh
      auto& p_front = mesh_mesh_viz.points.emplace_back();
      tf2::convert(graph_values.at<gtsam::Pose3>(front).translation(), p_front);
      auto& p_back = mesh_mesh_viz.points.emplace_back();
      tf2::convert(graph_values.at<gtsam::Pose3>(back).translation(), p_back);
      color.r = 1.0;
      color.g = 0.0;
      color.b = 0.0;
      color.a = 0.8;
      mesh_mesh_viz.colors.push_back(color);
      mesh_mesh_viz.colors.push_back(color);
    } else {
      // pose-to-mesh
      auto& p_front = pose_mesh_viz.points.emplace_back();
      tf2::convert(graph_values.at<gtsam::Pose3>(front).translation(), p_front);
      auto& p_back = pose_mesh_viz.points.emplace_back();
      tf2::convert(graph_values.at<gtsam::Pose3>(back).translation(), p_back);
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.2;
      color.a = 0.3;
      pose_mesh_viz.colors.push_back(color);
      pose_mesh_viz.colors.push_back(color);
    }
  }
}

}  // namespace kimera_pgmo
