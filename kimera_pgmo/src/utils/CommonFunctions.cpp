/**
 * @file   CommonFunctions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */
#include "kimera_pgmo/utils/CommonFunctions.h"

#include <gtsam/slam/BetweenFactor.h>
#include <pcl/conversions.h>
#include <pose_graph_tools_msgs/PoseGraphEdge.h>
#include <pose_graph_tools_msgs/PoseGraphNode.h>

#include <algorithm>
#include <chrono>
#include <limits>

namespace kimera_pgmo {

KimeraPgmoMesh PolygonMeshToPgmoMeshMsg(const size_t& id,
                                        const pcl::PolygonMesh& polygon_mesh,
                                        const std::vector<Timestamp>& vertex_timestamps,
                                        const std::string& frame_id) {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  pcl::fromPCLPointCloud2(polygon_mesh.cloud, vertices_cloud);

  return PolygonMeshToPgmoMeshMsg(
      id, vertices_cloud, polygon_mesh.polygons, vertex_timestamps, frame_id);
}

KimeraPgmoMesh PolygonMeshToPgmoMeshMsg(
    const size_t& id,
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const std::vector<pcl::Vertices>& polygons,
    const std::vector<Timestamp>& vertex_timestamps,
    const std::string& frame_id,
    const IndexMapping& vertex_index_mappings) {
  KimeraPgmoMesh new_mesh;
  if (vertices.size() == 0) {
    return new_mesh;
  }

  assert(vertices.size() == vertex_timestamps.size());

  // Convert vertices
  new_mesh.vertices.resize(vertices.size());
  new_mesh.vertex_colors.resize(vertices.size());
  new_mesh.vertex_stamps.resize(vertices.size());
  new_mesh.vertex_indices.resize(vertices.size());
  for (size_t i = 0; i < vertices.points.size(); i++) {
    geometry_msgs::Point p;
    p.x = vertices.points[i].x;
    p.y = vertices.points[i].y;
    p.z = vertices.points[i].z;
    new_mesh.vertices[i] = p;
    // Point color
    std_msgs::ColorRGBA color;
    constexpr float color_conv_factor = 1.0f / std::numeric_limits<uint8_t>::max();
    color.r = color_conv_factor * static_cast<float>(vertices.points[i].r);
    color.g = color_conv_factor * static_cast<float>(vertices.points[i].g);
    color.b = color_conv_factor * static_cast<float>(vertices.points[i].b);
    color.a = color_conv_factor * static_cast<float>(vertices.points[i].a);
    new_mesh.vertex_colors[i] = color;
    new_mesh.vertex_stamps[i].fromNSec(vertex_timestamps[i]);
    new_mesh.vertex_indices[i] = -1;
    if (vertex_index_mappings.count(i)) {
      new_mesh.vertex_indices[i] = vertex_index_mappings.at(i);
    }
  }

  // Convert polygons
  new_mesh.triangles.resize(polygons.size());
  for (size_t i = 0; i < polygons.size(); i++) {
    TriangleIndices triangle;
    triangle.vertex_indices[0] = polygons[i].vertices[0];
    triangle.vertex_indices[1] = polygons[i].vertices[1];
    triangle.vertex_indices[2] = polygons[i].vertices[2];
    new_mesh.triangles[i] = triangle;
  }

  new_mesh.header.frame_id = frame_id;
  new_mesh.id = id;
  new_mesh.header.stamp.fromNSec(vertex_timestamps.back());
  return new_mesh;
}

pcl::PolygonMesh PgmoMeshMsgToPolygonMesh(const KimeraPgmoMesh& mesh_msg,
                                          std::vector<Timestamp>* vertex_stamps,
                                          std::vector<int>* vertex_graph_indices) {
  pcl::PolygonMesh mesh;

  assert(mesh_msg.vertices.size() == mesh_msg.vertex_stamps.size());
  assert(nullptr != vertex_stamps);

  if (mesh_msg.vertices.size() == 0) {
    return mesh;
  }

  // Clear vertex stamps
  vertex_stamps->clear();
  vertex_graph_indices->clear();

  // Convert vertices
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  bool color = (mesh_msg.vertex_colors.size() == mesh_msg.vertices.size());
  constexpr float color_conv_factor = 1.0f * std::numeric_limits<uint8_t>::max();
  for (size_t i = 0; i < mesh_msg.vertices.size(); i++) {
    const geometry_msgs::Point& p = mesh_msg.vertices[i];
    pcl::PointXYZRGBA point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    if (color) {
      const std_msgs::ColorRGBA& c = mesh_msg.vertex_colors[i];
      point.r = static_cast<uint8_t>(color_conv_factor * c.r);
      point.g = static_cast<uint8_t>(color_conv_factor * c.g);
      point.b = static_cast<uint8_t>(color_conv_factor * c.b);
      point.a = static_cast<uint8_t>(color_conv_factor * c.a);
    }
    vertices_cloud.points.push_back(point);
    vertex_stamps->push_back(mesh_msg.vertex_stamps[i].toNSec());
    vertex_graph_indices->push_back(mesh_msg.vertex_indices[i]);
  }

  pcl::toPCLPointCloud2(vertices_cloud, mesh.cloud);

  // Convert polygons
  for (const TriangleIndices& triangle : mesh_msg.triangles) {
    pcl::Vertices polygon;
    for (size_t i = 0; i < 3; i++) {
      polygon.vertices.push_back(triangle.vertex_indices[i]);
    }
    mesh.polygons.push_back(polygon);
  }

  return mesh;
}

gtsam::Pose3 RosToGtsam(const geometry_msgs::Pose& transform) {
  gtsam::Pose3 pose;
  pose = gtsam::Pose3(
      gtsam::Rot3(transform.orientation.w,
                  transform.orientation.x,
                  transform.orientation.y,
                  transform.orientation.z),
      gtsam::Point3(transform.position.x, transform.position.y, transform.position.z));
  return pose;
}

geometry_msgs::Pose GtsamToRos(const gtsam::Pose3& pose) {
  const gtsam::Point3& translation = pose.translation();
  const gtsam::Quaternion& quaternion = pose.rotation().toQuaternion();

  geometry_msgs::Pose ros_pose;

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

pcl::PolygonMesh CombineMeshes(const pcl::PolygonMesh& mesh1,
                               const pcl::PolygonMesh& mesh2,
                               bool check_duplicate_vertices) {
  if (mesh1.polygons.size() == 0) {
    return mesh2;
  } else if (mesh2.polygons.size() == 0) {
    return mesh1;
  }

  pcl::PointCloud<pcl::PointXYZRGBA> vertices1, vertices2;
  pcl::fromPCLPointCloud2(mesh1.cloud, vertices1);
  pcl::fromPCLPointCloud2(mesh2.cloud, vertices2);

  pcl::PolygonMesh out_mesh;
  out_mesh.polygons = mesh1.polygons;

  // Iterate through the second set of vertices and remap indices
  size_t new_index = vertices1.points.size();
  const size_t orig_num_vertices = vertices1.points.size();
  std::vector<size_t> new_indices;
  for (size_t i = 0; i < vertices2.points.size(); i++) {
    // check if point duplicated
    bool new_point = true;
    size_t idx = new_index;
    if (check_duplicate_vertices) {
      for (size_t j = 0; j < vertices1.points.size(); j++) {
        if (vertices1.points[j].x == vertices2.points[i].x &&
            vertices1.points[j].y == vertices2.points[i].y &&
            vertices1.points[j].z == vertices2.points[i].z) {
          idx = j;
          new_point = false;
          vertices1.points[j] = vertices2.points[i];
          break;
        }
      }
    }
    new_indices.push_back(idx);
    if (new_point) {
      vertices1.push_back(vertices2.points[i]);
      new_index++;
    }
  }

  // Now iterate throught the polygons in mesh two and combine using new indices
  for (const pcl::Vertices& tri : mesh2.polygons) {
    pcl::Vertices new_triangle;
    bool to_add = false;
    for (size_t v : tri.vertices) {
      new_triangle.vertices.push_back(new_indices.at(v));
      if (new_indices.at(v) >= orig_num_vertices) to_add = true;
    }
    if (to_add) out_mesh.polygons.push_back(new_triangle);
  }
  pcl::toPCLPointCloud2(vertices1, out_mesh.cloud);
  return out_mesh;
}

pcl::PolygonMesh CombineMeshes(const pcl::PolygonMesh& mesh1,
                               const pcl::PolygonMesh& mesh2,
                               const std::vector<size_t>& indices_to_check,
                               std::vector<size_t>* vertex_indices) {
  if (mesh2.polygons.size() == 0) {
    return mesh1;
  }

  pcl::PointCloud<pcl::PointXYZRGBA> vertices1, vertices2;
  pcl::fromPCLPointCloud2(mesh1.cloud, vertices1);
  pcl::fromPCLPointCloud2(mesh2.cloud, vertices2);

  pcl::PolygonMesh out_mesh;
  out_mesh.polygons = mesh1.polygons;

  // Iterate through the second set of vertices and remap indices
  size_t new_index = vertices1.points.size();
  const size_t orig_num_vertices = vertices1.points.size();
  std::vector<size_t> new_indices;
  for (size_t i = 0; i < vertices2.points.size(); i++) {
    // check if point duplicated
    bool new_point = true;
    size_t idx = new_index;
    for (size_t j : indices_to_check) {
      if (vertices1.points[j].x == vertices2.points[i].x &&
          vertices1.points[j].y == vertices2.points[i].y &&
          vertices1.points[j].z == vertices2.points[i].z) {
        idx = j;
        new_point = false;
        vertices1.points[j] = vertices2.points[i];
        break;
      }
    }
    new_indices.push_back(idx);
    if (new_point) {
      vertices1.push_back(vertices2.points[i]);
      new_index++;
    }
  }

  // if no new points assume no new polygons
  if (new_index > orig_num_vertices) {
    // Iterate throught the polygons in mesh2 and combine using new indices
    for (const pcl::Vertices& tri : mesh2.polygons) {
      pcl::Vertices new_triangle;
      bool to_add = false;
      for (size_t v : tri.vertices) {
        new_triangle.vertices.push_back(new_indices.at(v));
        if (new_indices.at(v) >= orig_num_vertices) to_add = true;
      }
      // only push back if triangle has new vertices
      if (to_add) out_mesh.polygons.push_back(new_triangle);
    }
  }

  pcl::toPCLPointCloud2(vertices1, out_mesh.cloud);
  // track newly assigned indices
  *vertex_indices = new_indices;
  return out_mesh;
}

void AppendMesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mesh_vertices,
                std::shared_ptr<std::vector<pcl::Vertices>> mesh_faces,
                const pcl::PointCloud<pcl::PointXYZRGBA>& vertices_to_add,
                const std::vector<pcl::Vertices>& faces_to_add) {
  // Iterate through the second set of vertices and remap indices
  size_t new_index = mesh_vertices->size();
  std::vector<size_t> new_indices;
  for (size_t i = 0; i < vertices_to_add.size(); i++) {
    new_indices.push_back(new_index);
    mesh_vertices->push_back(vertices_to_add.points[i]);
    new_index++;
  }

  // Now iterate throught the polygons in mesh two and combine using new indices
  for (const pcl::Vertices& tri : faces_to_add) {
    pcl::Vertices new_triangle;
    for (size_t v : tri.vertices) {
      new_triangle.vertices.push_back(new_indices.at(v));
    }
    mesh_faces->push_back(new_triangle);
  }
}

bool PolygonsEqual(const pcl::Vertices& p1, const pcl::Vertices& p2) {
  std::vector<uint32_t> p1_v = p1.vertices;
  std::vector<uint32_t> p2_v = p2.vertices;
  if (p1_v.size() != p2_v.size()) return false;
  for (size_t i = 0; i < p1_v.size(); i++) {
    std::rotate(p1_v.begin(), p1_v.begin() + 1, p1_v.end());
    if (p1_v == p2_v) return true;
  }
  return false;
}

// Convert gtsam posegaph to PoseGraph msg
GraphMsgPtr GtsamGraphToRos(const gtsam::NonlinearFactorGraph& factors,
                            const gtsam::Values& values,
                            const std::map<size_t, std::vector<Timestamp>>& timestamps,
                            const gtsam::Vector& gnc_weights,
                            const std::string& frame_id) {
  std::vector<pose_graph_tools_msgs::PoseGraphEdge> edges;

  // first store the factors as edges
  for (size_t i = 0; i < factors.size(); i++) {
    const auto factor_ptr =
        dynamic_cast<const gtsam::BetweenFactor<gtsam::Pose3>*>(factors[i].get());
    // check if between factor
    if (factor_ptr) {
      // convert to between factor
      const auto& factor = *factor_ptr;
      // convert between factor to PoseGraphEdge type
      pose_graph_tools_msgs::PoseGraphEdge edge;
      edge.header.frame_id = frame_id;
      gtsam::Symbol front(factor.front());
      gtsam::Symbol back(factor.back());
      edge.key_from = front.index();
      edge.key_to = back.index();
      edge.robot_from = robot_prefix_to_id.at(front.chr());
      edge.robot_to = robot_prefix_to_id.at(back.chr());

      if (edge.key_to == edge.key_from + 1 &&
          edge.robot_from == edge.robot_to) {  // check if odom
        edge.type = pose_graph_tools_msgs::PoseGraphEdge::ODOM;
        try {
          edge.header.stamp.fromNSec(timestamps.at(edge.robot_to).at(edge.key_to));
        } catch (...) {
          // ignore
        }

      } else {
        if (gnc_weights.size() > i && gnc_weights(i) < 0.5) {
          edge.type = pose_graph_tools_msgs::PoseGraphEdge::REJECTED_LOOPCLOSE;
        } else {
          edge.type = pose_graph_tools_msgs::PoseGraphEdge::LOOPCLOSE;
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
      for (size_t i = 0; i < edge.covariance.size(); i++) {
        size_t row = static_cast<size_t>(i / 6);
        size_t col = i % 6;
        edge.covariance[i] = covariance(row, col);
      }
      edges.push_back(edge);
    }
  }

  std::vector<pose_graph_tools_msgs::PoseGraphNode> nodes;
  // Then store the values as nodes
  gtsam::KeyVector key_list = values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    gtsam::Symbol node_symb(key_list[i]);
    if (robot_prefix_to_id.count(node_symb.chr())) {
      const size_t robot_id = robot_prefix_to_id.at(node_symb.chr());

      pose_graph_tools_msgs::PoseGraphNode node;
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
        ROS_WARN_ONCE(
            "Invalid timestamp for trajectory poses when converting to "
            "PoseGraph msg. ");
      } else {
        node.header.stamp.fromNSec(timestamps.at(robot_id).at(node_symb.index()));
      }

      nodes.push_back(node);
    }
  }

  pose_graph_tools_msgs::PoseGraph::Ptr posegraph(
      new pose_graph_tools_msgs::PoseGraph());
  posegraph->header.stamp = ros::Time::now();
  posegraph->header.frame_id = frame_id;
  posegraph->nodes = nodes;
  posegraph->edges = edges;
  return posegraph;
}

bool SurfaceExists(const pcl::Vertices& new_surface,
                   const std::map<size_t, std::vector<size_t>>& adjacent_surfaces,
                   const std::vector<pcl::Vertices>& surfaces) {
  // Degenerate face
  if (new_surface.vertices.size() < 3) return false;

  const size_t idx0 = new_surface.vertices.at(0);
  if (adjacent_surfaces.find(idx0) == adjacent_surfaces.end()) {
    // vertex not in adjacent surfaces
    return false;
  }

  for (const auto& s_idx : adjacent_surfaces.at(idx0)) {
    pcl::Vertices p = surfaces.at(s_idx);
    if (p.vertices == new_surface.vertices) {
      return true;
    }
  }
  return false;
}
}  // namespace kimera_pgmo
