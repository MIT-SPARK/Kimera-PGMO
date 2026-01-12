/**
 * @file   mesh_conversion.cpp
 * @brief  Conversion to and from ROS types
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/conversion/mesh.h"

#include <kimera_pgmo/mesh_types.h>
#include <kimera_pgmo/pcl_mesh_traits.h>
#include <pcl/conversions.h>

#include <rclcpp/time.hpp>

namespace kimera_pgmo::conversions {

using traits::Timestamp;

Mesh::UniquePtr toMsg(size_t robot_id,
                      const pcl::PolygonMesh& mesh,
                      const std::vector<Timestamp>& stamps,
                      const std::string& frame_id,
                      const IndexMapping* graph_indices) {
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  return toMsg(robot_id, cloud, mesh.polygons, stamps, frame_id, graph_indices);
}

Mesh::UniquePtr toMsg(size_t robot_id,
                      const pcl::PointCloud<pcl::PointXYZRGBA>& cloud,
                      const std::vector<pcl::Vertices>& faces,
                      const std::vector<Timestamp>& stamps,
                      const std::string& frame_id,
                      const IndexMapping* graph_indices) {
  if (cloud.size() != stamps.size()) {
    SPARK_LOG(ERROR) << "Vertices and timestamps disagree: points=" << cloud.size()
                     << " != stamps=" << stamps.size();
    return nullptr;
  }

  std_msgs::msg::Header header;
  header.frame_id = frame_id;
  if (!stamps.empty()) {
    header.stamp = rclcpp::Time(stamps.back());
  }

  ConstStampedCloud vertices{cloud, stamps};
  return toMsg(robot_id, vertices, faces, header, graph_indices);
}

pcl::PolygonMesh fromMsg(const Mesh& msg,
                         std::vector<Timestamp>* vertex_stamps,
                         std::vector<int>* vertex_graph_indices) {
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  if (vertex_stamps) {
    StampedCloud vertices{vertices_cloud, *vertex_stamps};
    fillFromMsg(msg, vertices, mesh.polygons, vertex_graph_indices);
  } else {
    fillFromMsg(msg, vertices_cloud, mesh.polygons, vertex_graph_indices);
  }

  pcl::toPCLPointCloud2(vertices_cloud, mesh.cloud);
  return mesh;
}

}  // namespace kimera_pgmo::conversions
