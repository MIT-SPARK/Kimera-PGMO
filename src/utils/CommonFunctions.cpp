/**
 * @file   CommonFunctions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */
#include <algorithm>
#include <chrono>
#include <limits>

#include <geometry_msgs/Point.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mesh_msgs/TriangleIndices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <voxblox_msgs/MeshBlock.h>

#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/tinyply.h"

namespace kimera_pgmo {

void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh) {
  if (NULL == mesh) {
    std::cout << "Mesh is NULL in ReadMeshFromPly" << std::endl;
    return;
  }

  std::cout << "filename: " << filename << std::endl;
  std::unique_ptr<std::istream> file_stream;
  file_stream.reset(new std::ifstream(filename, std::ios::binary));
  if (!file_stream || file_stream->fail())
    throw std::runtime_error("file_stream failed to open " + filename);

  file_stream->seekg(0, std::ios::end);
  const float size_mb = file_stream->tellg() * float(1e-6);
  file_stream->seekg(0, std::ios::beg);

  tinyply::PlyFile file;
  file.parse_header(*file_stream);

  std::cout << "\t[ply_header] Type: "
            << (file.is_binary_file() ? "binary" : "ascii") << std::endl;
  for (const auto& c : file.get_comments())
    std::cout << "\t[ply_header] Comment: " << c << std::endl;
  for (const auto& c : file.get_info())
    std::cout << "\t[ply_header] Info: " << c << std::endl;

  for (const auto& e : file.get_elements()) {
    std::cout << "\t[ply_header] element: " << e.name << " (" << e.size << ")"
              << std::endl;
    for (const auto& p : e.properties) {
      std::cout << "\t[ply_header] \tproperty: " << p.name
                << " (type=" << tinyply::PropertyTable[p.propertyType].str
                << ")";
      if (p.isList)
        std::cout << " (list_type=" << tinyply::PropertyTable[p.listType].str
                  << ")";
      std::cout << std::endl;
    }
  }

  std::shared_ptr<tinyply::PlyData> vertices, colors, faces;

  // The header information can be used to programmatically extract properties
  // on elements known to exist in the header prior to reading the data. For
  // brevity of this sample, properties like vertex position are hard-coded:
  try {
    vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
  } catch (const std::exception& e) {
    std::cerr << "tinyply exception: " << e.what() << std::endl;
  }

  try {
    colors = file.request_properties_from_element(
        "vertex", {"red", "green", "blue", "alpha"});
  } catch (const std::exception& e) {
    std::cerr << "tinyply exception: " << e.what() << std::endl;
  }

  // Providing a list size hint (the last argument) is a 2x performance
  // improvement. If you have arbitrary ply files, it is best to leave this 0.
  try {
    faces = file.request_properties_from_element("face", {"vertex_indices"}, 3);
  } catch (const std::exception& e) {
    std::cerr << "tinyply exception: " << e.what() << std::endl;
  }

  file.read(*file_stream);

  // Extract vertices data
  const size_t numVerticesBytes = vertices->buffer.size_bytes();
  std::vector<float[3]> verts(vertices->count);
  std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);

  std::vector<float[4]> colrs;
  if (colors) {
    colrs = std::vector<float[4]>(colors->count);
    const size_t numColorBytes = colors->buffer.size_bytes();
    std::memcpy(colrs.data(), colors->buffer.get(), numColorBytes);
  }

  // Convert to pointcloud
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  for (size_t i = 0; i < verts.size(); i++) {
    pcl::PointXYZRGBA pcl_pt;
    pcl_pt.x = verts[i][0];
    pcl_pt.y = verts[i][1];
    pcl_pt.z = verts[i][2];
    if (colors) {
      pcl_pt.r = colrs[i][0];
      pcl_pt.g = colrs[i][1];
      pcl_pt.b = colrs[i][2];
      pcl_pt.a = colrs[i][3];
    }
    vertices_cloud.points.push_back(pcl_pt);
  }
  pcl::toPCLPointCloud2(vertices_cloud, mesh->cloud);

  // Extract surface data
  const size_t numFacesByets = faces->buffer.size_bytes();
  std::vector<int[3]> triangles(faces->count);
  std::memcpy(triangles.data(), faces->buffer.get(), numFacesByets);
  // Convert to polygon mesh type
  for (auto t : triangles) {
    pcl::Vertices tri;
    tri.vertices.push_back(t[0]);
    tri.vertices.push_back(t[1]);
    tri.vertices.push_back(t[2]);
    mesh->polygons.push_back(tri);
  }
}

void WriteMeshToPly(const std::string& filename, const pcl::PolygonMesh& mesh) {
  // Recast data
  // Extract surface data
  struct uint3 {
    uint32_t x, y, z;
  };
  struct float3 {
    float x, y, z;
  };
  struct rgba {
    uint8_t r, g, b, a;
  };

  std::vector<uint3> triangles;
  // Convert to polygon mesh type
  for (pcl::Vertices tri : mesh.polygons) {
    uint3 triangle;
    triangles.push_back({tri.vertices[0], tri.vertices[1], tri.vertices[2]});
  }
  // Get point cloud
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices_cloud);

  std::vector<float3> vertices;
  std::vector<rgba> colors;
  for (pcl::PointXYZRGBA p : vertices_cloud) {
    vertices.push_back({p.x, p.y, p.z});
    colors.push_back({p.r, p.g, p.b, p.a});
  }

  std::filebuf fb_ascii;
  fb_ascii.open(filename, std::ios::out);
  std::ostream outstream_ascii(&fb_ascii);
  if (outstream_ascii.fail())
    throw std::runtime_error("failed to open " + filename);

  tinyply::PlyFile output_file;

  output_file.add_properties_to_element(
      "vertex",
      {"x", "y", "z"},
      tinyply::Type::FLOAT32,
      vertices.size(),
      reinterpret_cast<uint8_t*>(vertices.data()),
      tinyply::Type::INVALID,
      0);

  output_file.add_properties_to_element(
      "vertex",
      {"red", "green", "blue", "alpha"},
      tinyply::Type::UINT8,
      colors.size(),
      reinterpret_cast<uint8_t*>(colors.data()),
      tinyply::Type::INVALID,
      0);

  output_file.add_properties_to_element(
      "face",
      {"vertex_indices"},
      tinyply::Type::UINT32,
      triangles.size(),
      reinterpret_cast<uint8_t*>(triangles.data()),
      tinyply::Type::UINT8,
      3);

  // Write an ASCII file
  output_file.write(outstream_ascii, false);
}

mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(
    const pcl::PolygonMesh& polygon_mesh) {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  pcl::fromPCLPointCloud2(polygon_mesh.cloud, vertices_cloud);

  mesh_msgs::TriangleMesh new_mesh =
      PolygonMeshToTriangleMeshMsg(vertices_cloud, polygon_mesh.polygons);

  return new_mesh;
}

mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const std::vector<pcl::Vertices>& polygons) {
  mesh_msgs::TriangleMesh new_mesh;
  if (vertices.size() == 0) return new_mesh;
  // Convert vertices
  for (size_t i = 0; i < vertices.points.size(); i++) {
    geometry_msgs::Point p;
    p.x = vertices.points[i].x;
    p.y = vertices.points[i].y;
    p.z = vertices.points[i].z;
    new_mesh.vertices.push_back(p);
    // Point color
    std_msgs::ColorRGBA color;
    constexpr float color_conv_factor =
        1.0f / std::numeric_limits<uint8_t>::max();
    color.r = color_conv_factor * static_cast<float>(vertices.points[i].r);
    color.g = color_conv_factor * static_cast<float>(vertices.points[i].g);
    color.b = color_conv_factor * static_cast<float>(vertices.points[i].b);
    color.a = color_conv_factor * static_cast<float>(vertices.points[i].a);
    new_mesh.vertex_colors.push_back(color);
  }

  // Convert polygons
  for (pcl::Vertices polygon : polygons) {
    mesh_msgs::TriangleIndices triangle;
    triangle.vertex_indices[0] = polygon.vertices[0];
    triangle.vertex_indices[1] = polygon.vertices[1];
    triangle.vertex_indices[2] = polygon.vertices[2];

    new_mesh.triangles.push_back(triangle);
  }

  return new_mesh;
}

pcl::PolygonMesh TriangleMeshMsgToPolygonMesh(
    const mesh_msgs::TriangleMesh& mesh_msg) {
  pcl::PolygonMesh mesh;
  if (mesh_msg.vertices.size() == 0) return mesh;
  // Convert vertices
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  bool color = (mesh_msg.vertex_colors.size() == mesh_msg.vertices.size());
  constexpr float color_conv_factor =
      1.0f * std::numeric_limits<uint8_t>::max();
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
  }
  pcl::toPCLPointCloud2(vertices_cloud, mesh.cloud);
  // Convert polygons
  for (mesh_msgs::TriangleIndices triangle : mesh_msg.triangles) {
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
      gtsam::Point3(
          transform.position.x, transform.position.y, transform.position.z));
  return pose;
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
  for (pcl::Vertices tri : mesh2.polygons) {
    pcl::Vertices new_triangle;
    for (size_t v : tri.vertices) {
      new_triangle.vertices.push_back(new_indices.at(v));
    }
    out_mesh.polygons.push_back(new_triangle);
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
    for (pcl::Vertices tri : mesh2.polygons) {
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
GraphMsgPtr GtsamGraphToRos(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values,
    const std::map<size_t, std::vector<ros::Time> >& timestamps) {
  std::vector<pose_graph_tools::PoseGraphEdge> edges;

  // first store the factors as edges
  for (size_t i = 0; i < factors.size(); i++) {
    // check if between factor
    if (boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
            factors[i])) {
      // convert to between factor
      const gtsam::BetweenFactor<gtsam::Pose3>& factor =
          *boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3> >(
              factors[i]);
      // convert between factor to PoseGraphEdge type
      pose_graph_tools::PoseGraphEdge edge;
      edge.header.stamp = ros::Time::now();
      edge.header.frame_id = "world";
      gtsam::Symbol front(factor.front());
      gtsam::Symbol back(factor.back());
      edge.key_from = front.index();
      edge.key_to = back.index();
      edge.robot_from = robot_prefix_to_id.at(front.chr());
      edge.robot_to = robot_prefix_to_id.at(back.chr());

      if (edge.key_to == edge.key_from + 1 &&
          edge.robot_from == edge.robot_to) {  // check if odom
        edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
        try {
          edge.header.stamp = timestamps.at(edge.robot_to).at(edge.key_to);
        } catch (...) {
          // ignore
        }

      } else {
        edge.type = pose_graph_tools::PoseGraphEdge::LOOPCLOSE;
      }
      // transforms - translation
      const gtsam::Point3& translation = factor.measured().translation();
      edge.pose.position.x = translation.x();
      edge.pose.position.y = translation.y();
      edge.pose.position.z = translation.z();
      // transforms - rotation (to quaternion)
      const gtsam::Quaternion& quaternion =
          factor.measured().rotation().toQuaternion();
      edge.pose.orientation.x = quaternion.x();
      edge.pose.orientation.y = quaternion.y();
      edge.pose.orientation.z = quaternion.z();
      edge.pose.orientation.w = quaternion.w();

      // transfer covariance
      gtsam::Matrix66 covariance =
          boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
              factor.noiseModel())
              ->covariance();
      for (size_t i = 0; i < edge.covariance.size(); i++) {
        size_t row = static_cast<size_t>(i / 6);
        size_t col = i % 6;
        edge.covariance[i] = covariance(row, col);
      }
      edges.push_back(edge);
    }
  }

  std::vector<pose_graph_tools::PoseGraphNode> nodes;
  // Then store the values as nodes
  gtsam::KeyVector key_list = values.keys();
  for (size_t i = 0; i < key_list.size(); i++) {
    gtsam::Symbol node_symb(key_list[i]);
    try {
      const size_t robot_id = robot_prefix_to_id.at(node_symb.chr());

      pose_graph_tools::PoseGraphNode node;
      node.header.stamp = ros::Time::now();
      node.header.frame_id = "world";
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

      node.header.stamp = timestamps.at(robot_id).at(node_symb.index());

      nodes.push_back(node);
    } catch (...) {
      // ignore
    }
  }

  pose_graph_tools::PoseGraph posegraph;
  posegraph.header.stamp = ros::Time::now();
  posegraph.header.frame_id = "world";
  posegraph.nodes = nodes;
  posegraph.edges = edges;
  return boost::make_shared<pose_graph_tools::PoseGraph>(posegraph);
}

bool SurfaceExists(
    const pcl::Vertices& new_surface,
    const std::vector<std::vector<pcl::Vertices> >& adjacent_surfaces) {
  // Degenerate face
  if (new_surface.vertices.size() < 3) return false;

  const size_t idx0 = new_surface.vertices.at(0);
  bool exist = false;
  if (idx0 > adjacent_surfaces.size()) {
    // vertex not yet tracked in adjacent surfaces
    return false;
  }

  for (pcl::Vertices p : adjacent_surfaces[idx0]) {
    if (p.vertices == new_surface.vertices) {
      return true;
    }
  }
  return false;
}
}  // namespace kimera_pgmo