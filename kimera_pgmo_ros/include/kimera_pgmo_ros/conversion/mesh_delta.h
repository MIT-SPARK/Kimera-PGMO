/**
 * @file   mesh_delta_conversion.h
 * @brief  Conversion to/from ROS
 * @author Nathan Hughes
 * @author Yun Chang
 */

#pragma once

#include <kimera_pgmo/mesh_delta.h>

#include <type_traits>

#include <kimera_pgmo_msgs/msg/mesh_delta.hpp>
#include <rclcpp/publisher.hpp>
#include <rclcpp/subscription.hpp>
#include <rclcpp/type_adapter.hpp>

namespace rclcpp {

template <>
struct TypeAdapter<kimera_pgmo::MeshDelta, kimera_pgmo_msgs::msg::MeshDelta> {
  using is_specialized = std::true_type;
  using custom_type = kimera_pgmo::MeshDelta;
  using ros_message_type = kimera_pgmo_msgs::msg::MeshDelta;

  static void convert_to_ros_message(const custom_type& src, ros_message_type& dest);
  static void convert_to_custom(const ros_message_type& src, custom_type& dest);
};

}  // namespace rclcpp

namespace kimera_pgmo {

using MeshDeltaTypeAdapter =
    rclcpp::adapt_type<MeshDelta>::as<kimera_pgmo_msgs::msg::MeshDelta>;
using PgmoMeshDeltaPublisher = rclcpp::Publisher<MeshDeltaTypeAdapter>::SharedPtr;
using PgmoMeshDeltaSubscription = rclcpp::Subscription<MeshDeltaTypeAdapter>::SharedPtr;

}  // namespace kimera_pgmo
