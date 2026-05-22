#include "kimera_pgmo_ros/visualization_functions.h"

#include <kimera_pgmo/utils/common_functions.h>

#include <gtsam/nonlinear/NonlinearFactor.h>

#include <tf2_eigen/tf2_eigen.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace kimera_pgmo {

using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

namespace {

struct MeshEdge {
  geometry_msgs::msg::Point p_front;
  geometry_msgs::msg::Point p_back;
  double variance;
};

}  // namespace

void fillDeformationGraphMarkers(const DeformationGraph& graph,
                                 const rclcpp::Time& stamp,
                                 MarkerArray& mesh_mesh_viz,
                                 Marker& pose_mesh_viz,
                                 const std::string& frame_id) {
  auto graph_values = graph.getValues();
  auto graph_factors = graph.getFactors();

  // header for the pose to mesh edges
  pose_mesh_viz.header.frame_id = frame_id;
  pose_mesh_viz.header.stamp = stamp;
  pose_mesh_viz.id = 0;
  pose_mesh_viz.action = Marker::ADD;
  pose_mesh_viz.type = Marker::LINE_LIST;
  pose_mesh_viz.scale.x = 0.02;

  // First pass: collect mesh-mesh edges (colored by factor variance below).
  std::vector<MeshEdge> mesh_edges;
  double min_var = std::numeric_limits<double>::max();
  double max_var = 0.0;

  for (size_t factor_idx = 0; factor_idx < graph_factors->size(); ++factor_idx) {
    const auto& factor = (*graph_factors)[factor_idx];
    if (!factor || factor->keys().size() != 2) {
      continue;
    }

    const gtsam::Symbol& front = factor->front();
    const gtsam::Symbol& back = factor->back();

    const bool front_is_pose =
        (robot_prefix_to_id.find(front.chr()) != robot_prefix_to_id.end());
    const bool back_is_pose =
        (robot_prefix_to_id.find(back.chr()) != robot_prefix_to_id.end());

    if (front_is_pose && back_is_pose) {
      continue;
    }

    if (!graph_values->exists(front) || !graph_values->exists(back)) {
      continue;
    }

    if (!front_is_pose && !back_is_pose) {
      // mesh-to-mesh: collect with variance and type
      MeshEdge edge;
      tf2::convert(graph_values->at<gtsam::Pose3>(front).translation(), edge.p_front);
      tf2::convert(graph_values->at<gtsam::Pose3>(back).translation(), edge.p_back);

      edge.variance = 1e-4;
      auto noise_factor =
          boost::dynamic_pointer_cast<gtsam::NoiseModelFactor>(factor);
      if (noise_factor && noise_factor->noiseModel()) {
        auto gaussian = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
            noise_factor->noiseModel());
        if (gaussian) {
          edge.variance = gaussian->covariance()(0, 0);
        }
      }

      min_var = std::min(min_var, edge.variance);
      max_var = std::max(max_var, edge.variance);
      mesh_edges.push_back(edge);
    } else {
      auto& p_front = pose_mesh_viz.points.emplace_back();
      tf2::convert(graph_values->at<gtsam::Pose3>(front).translation(), p_front);
      auto& p_back = pose_mesh_viz.points.emplace_back();
      tf2::convert(graph_values->at<gtsam::Pose3>(back).translation(), p_back);
      std_msgs::msg::ColorRGBA color;
      color.r = 1.0;
      color.g = 1.0;
      color.b = 0.2;
      color.a = 0.3;
      pose_mesh_viz.colors.push_back(color);
      pose_mesh_viz.colors.push_back(color);
    }
  }

  constexpr double kEdgeWidth = 0.025;
  const double log_min = (min_var > 0) ? std::log(min_var) : -12.0;
  const double log_max = (max_var > 0) ? std::log(max_var) : -8.0;
  const double log_range = log_max - log_min;

  {
    Marker clear_marker;
    clear_marker.action = Marker::DELETEALL;
    clear_marker.header.frame_id = frame_id;
    clear_marker.header.stamp = stamp;
    clear_marker.ns = "mesh_mesh_edges";
    mesh_mesh_viz.markers.push_back(clear_marker);
  }

  int mesh_edge_id = 1;

  for (const auto& edge : mesh_edges) {
    Marker m;
    m.header.frame_id = frame_id;
    m.header.stamp = stamp;
    m.action = Marker::ADD;
    m.type = Marker::LINE_LIST;
    m.ns = "mesh_mesh_edges";
    m.id = mesh_edge_id++;
    m.scale.x = kEdgeWidth;
    m.points.push_back(edge.p_front);
    m.points.push_back(edge.p_back);

    // Color by factor variance: low (stiff) -> green, high (loose) -> red.
    const double log_var = (edge.variance > 0) ? std::log(edge.variance) : log_min;
    const double t = (log_range > 0) ? (log_var - log_min) / log_range : 0.5;
    std_msgs::msg::ColorRGBA color;
    color.r = static_cast<float>(t);
    color.g = static_cast<float>(1.0 - t);
    color.b = 0.0f;
    color.a = 0.8f;

    m.colors.push_back(color);
    m.colors.push_back(color);
    mesh_mesh_viz.markers.push_back(m);
  }
}

}  // namespace kimera_pgmo
