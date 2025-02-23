/**
 * @file   common_functions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <kimera_pgmo/mesh_types.h>

#include <map>
#include <string>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <pose_graph_tools_msgs/msg/pose_graph.hpp>
#include <rclcpp/type_adapter.hpp>

namespace rclcpp {

template <>
struct TypeAdapter<gtsam::Pose3, geometry_msgs::msg::Pose> {
  using is_specialized = std::true_type;
  using custom_type = gtsam::Pose3;
  using ros_message_type = geometry_msgs::msg::Pose;

  /*! \brief Converts a gtsam pose type to ros geometry message
   *  - pose: gtsam pose3
   *  - outputs pose as geometry_msg/pose
   */
  static void convert_to_ros_message(const custom_type& src, ros_message_type& dest);

  /*! \brief Converts a ros pose type to gtsam Pose3
   *  - transform: ros geometry_msgs pose type
   *  - outputs pose as gtsam Pose3
   */
  static void convert_to_custom(const ros_message_type& src, custom_type& dest);
};

}  // namespace rclcpp

namespace kimera_pgmo::conversions {

using PoseTypeAdapter = rclcpp::adapt_type<gtsam::Pose3>::as<geometry_msgs::msg::Pose>;
using GraphMsgPtr = pose_graph_tools_msgs::msg::PoseGraph::ConstSharedPtr;
using kimera_pgmo::traits::Timestamp;

/*! \brief Convert gtsam factor graph and estimated values to ros graph msg for
 * publication
 *  - factors: gtsam NonlinearFactorGraph storing all added factors
 *  - values: gtsam Values storing estimated values
 *  - timestamps: map from robot id to timestamps of the poses in the
 * trajectories
 *  - gnc_weights: weights obtained from gnc optimizer for inlier-outlier
 * sorting
 */
GraphMsgPtr GtsamGraphToRos(const gtsam::NonlinearFactorGraph& factors,
                            const gtsam::Values& values,
                            const std::map<size_t, std::vector<Timestamp>>& timestamps,
                            const gtsam::Vector& gnc_weights = gtsam::Vector(),
                            const std::string& frame_id = "world");

}  // namespace kimera_pgmo::conversions
