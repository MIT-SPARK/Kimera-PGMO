/**
 * @file   Conversions.cpp
 * @brief  Conversion to and from ROS types
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/conversion/gtsam.h"

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <kimera_pgmo/mesh_traits.h>
#include <kimera_pgmo/utils/common_functions.h>
#include <kimera_pgmo/utils/logging.h>

#include <rclcpp/clock.hpp>
#include <rclcpp/time.hpp>

namespace kimera_pgmo::conversions {

using pose_graph_tools_msgs::msg::PoseGraph;
using pose_graph_tools_msgs::msg::PoseGraphEdge;
using pose_graph_tools_msgs::msg::PoseGraphNode;

gtsam::Pose3 RosToGtsam(const geometry_msgs::msg::Pose& transform) {
  gtsam::Pose3 pose;
  pose = gtsam::Pose3(
      gtsam::Rot3(transform.orientation.w,
                  transform.orientation.x,
                  transform.orientation.y,
                  transform.orientation.z),
      gtsam::Point3(transform.position.x, transform.position.y, transform.position.z));
  return pose;
}

geometry_msgs::msg::Pose GtsamToRos(const gtsam::Pose3& pose) {
  const gtsam::Point3& translation = pose.translation();
  const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();

  geometry_msgs::msg::Pose ros_pose;

  // pose - translation
  ros_pose.position.x = translation.x();
  ros_pose.position.y = translation.y();
  ros_pose.position.z = translation.z();
  // pose - rotation (to quaternion)
  ros_pose.orientation.x = quaternion.x();
  ros_pose.orientation.y = quaternion.y();
  ros_pose.orientation.z = quaternion.z();
  ros_pose.orientation.w = quaternion.w();

  return ros_pose;
}

// Convert gtsam posegaph to PoseGraph msg
GraphMsgPtr GtsamGraphToRos(const gtsam::NonlinearFactorGraph& factors,
                            const gtsam::Values& values,
                            const std::map<size_t, std::vector<Timestamp>>& timestamps,
                            const gtsam::Vector& gnc_weights,
                            const std::string& frame_id) {
  std::vector<PoseGraphEdge> edges;

  // first store the factors as edges
  for (size_t i = 0; i < factors.size(); i++) {
    const auto factor_ptr =
        dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(factors[i].get());
    // check if between factor
    if (factor_ptr) {
      // convert to between factor
      const auto& factor = *factor_ptr;
      // convert between factor to PoseGraphEdge type
      PoseGraphEdge edge;
      edge.header.frame_id = frame_id;
      gtsam::Symbol front(factor.front());
      gtsam::Symbol back(factor.back());
      edge.key_from = front.index();
      edge.key_to = back.index();
      edge.robot_from = robot_prefix_to_id.at(front.chr());
      edge.robot_to = robot_prefix_to_id.at(back.chr());

      if (edge.key_to == edge.key_from + 1 &&
          edge.robot_from == edge.robot_to) {  // check if odom
        edge.type = PoseGraphEdge::ODOM;
        try {
          edge.header.stamp =
              rclcpp::Time(timestamps.at(edge.robot_to).at(edge.key_to));
        } catch (...) {
          // ignore
        }

      } else {
        if (static_cast<size_t>(gnc_weights.size()) > i && gnc_weights(i) < 0.5) {
          edge.type = PoseGraphEdge::REJECTED_LOOPCLOSE;
        } else {
          edge.type = PoseGraphEdge::LOOPCLOSE;
        }
      }
      // transforms - translation
      const gtsam::Point3& translation = factor.measured().translation();
      edge.pose.position.x = translation.x();
      edge.pose.position.y = translation.y();
      edge.pose.position.z = translation.z();
      // transforms - rotation (to quaternion)
      const gtsam::Quaternion& quaternion = factor.measured().rotation().toQuaternion();
      edge.pose.orientation.x = quaternion.x();
      edge.pose.orientation.y = quaternion.y();
      edge.pose.orientation.z = quaternion.z();
      edge.pose.orientation.w = quaternion.w();

      // transfer covariance
      gtsam::Matrix66 covariance =
          dynamic_cast<const gtsam::noiseModel::Gaussian*>(factor.noiseModel().get())
              ->covariance();
      for (size_t j = 0; j < edge.covariance.size(); j++) {
        size_t row = static_cast<size_t>(j / 6);
        size_t col = j % 6;
        edge.covariance[j] = covariance(row, col);
      }
      edges.push_back(edge);
    }
  }

  std::vector<PoseGraphNode> nodes;
  // Then store the values as nodes
  gtsam::KeyVector key_list = values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    gtsam::Symbol node_symb(key_list[i]);
    if (robot_prefix_to_id.count(node_symb.chr())) {
      const size_t robot_id = robot_prefix_to_id.at(node_symb.chr());

      PoseGraphNode node;
      node.header.frame_id = frame_id;
      node.key = node_symb.index();
      node.robot_id = robot_id;
      const gtsam::Pose3& value = values.at<gtsam::Pose3>(key_list[i]);
      const gtsam::Point3& translation = value.translation();
      const gtsam::Quaternion& quaternion = value.rotation().toQuaternion();

      // pose - translation
      node.pose.position.x = translation.x();
      node.pose.position.y = translation.y();
      node.pose.position.z = translation.z();
      // pose - rotation (to quaternion)
      node.pose.orientation.x = quaternion.x();
      node.pose.orientation.y = quaternion.y();
      node.pose.orientation.z = quaternion.z();
      node.pose.orientation.w = quaternion.w();

      if (timestamps.count(robot_id) == 0 ||
          timestamps.at(robot_id).size() <= node_symb.index()) {
        SPARK_LOG(WARNING) << "Invalid timestamp for trajectory poses when converting "
                              "to PoseGraph msg.";
      } else {
        node.header.stamp = rclcpp::Time(timestamps.at(robot_id).at(node_symb.index()));
      }

      nodes.push_back(node);
    }
  }

  auto posegraph = std::make_shared<PoseGraph>();
  posegraph->header.stamp = rclcpp::Clock().now();
  posegraph->header.frame_id = frame_id;
  posegraph->nodes = nodes;
  posegraph->edges = edges;
  return posegraph;
}

}  // namespace kimera_pgmo::conversions

namespace rclcpp {

void TypeAdapter<gtsam::Pose3, geometry_msgs::msg::Pose>::convert_to_ros_message(
    const custom_type& src, ros_message_type& dest) {
  dest = kimera_pgmo::conversions::GtsamToRos(src);
}

void TypeAdapter<gtsam::Pose3, geometry_msgs::msg::Pose>::convert_to_custom(
    const ros_message_type& src, custom_type& dest) {
  dest = kimera_pgmo::conversions::RosToGtsam(src);
}

}  // namespace rclcpp
