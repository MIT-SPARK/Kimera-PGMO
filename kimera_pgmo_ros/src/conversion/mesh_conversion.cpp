/**
 * @file   mesh_conversion.cpp
 * @brief  Conversion to and from ROS types
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/conversion/mesh_conversion.h"

#include <kimera_pgmo/pcl_mesh_traits.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

namespace kimera_pgmo::conversions {

using kimera_pgmo_msgs::KimeraPgmoMesh;
using kimera_pgmo_msgs::TriangleIndices;

KimeraPgmoMesh::Ptr toMsg(size_t robot_id,
                          const pcl::PolygonMesh& mesh,
                          const std::vector<Timestamp>& stamps,
                          const std::string& frame_id,
                          const IndexMapping* index_mapping) {
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  return toMsg(robot_id, cloud, mesh.polygons, stamps, frame_id, index_mapping);
}

KimeraPgmoMesh::Ptr toMsg(size_t robot_id,
                          const pcl::PointCloud<pcl::PointXYZRGBA>& cloud,
                          const std::vector<pcl::Vertices>& faces,
                          const std::vector<Timestamp>& stamps,
                          const std::string& frame_id,
                          const IndexMapping* index_mapping) {
  std_msgs::Header header;
  header.frame_id = frame_id;
  if (!stamps.empty()) {
    header.stamp.fromNSec(stamps.back());
  }

  ConstStampedCloud vertices{cloud, stamps};
  return toMsg(robot_id, vertices, faces, index_mapping, header);
}

pcl::PolygonMesh fromMsg(const KimeraPgmoMesh& msg,
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
