/**
 * @file   CommonFunctions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */
#include <algorithm>

#include <geometry_msgs/Point.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mesh_msgs/TriangleIndices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pose_graph_tools/PoseGraphEdge.h>
#include <pose_graph_tools/PoseGraphNode.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_msgs/ColorRGBA.h>
#include <voxblox_msgs/MeshBlock.h>
#include <pcl/impl/point_types.hpp>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/tinyply.h"

namespace mesher_mapper {

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
    colors =
        file.request_properties_from_element("vertex", {"r", "g", "b", "a"});
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
  struct double3 {
    double x, y, z;
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

  std::vector<double3> vertices;
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
      tinyply::Type::FLOAT64,
      vertices.size(),
      reinterpret_cast<uint8_t*>(vertices.data()),
      tinyply::Type::INVALID,
      0);

  output_file.add_properties_to_element(
      "vertex",
      {"r", "g", "b", "a"},
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

  mesh_msgs::TriangleMesh new_mesh;
  // Convert vertices
  for (size_t i = 0; i < vertices_cloud.points.size(); i++) {
    geometry_msgs::Point p;
    p.x = vertices_cloud.points[i].x;
    p.y = vertices_cloud.points[i].y;
    p.z = vertices_cloud.points[i].z;
    new_mesh.vertices.push_back(p);
    if (vertices_cloud.points[i].r) {
      std_msgs::ColorRGBA color;
      color.r = vertices_cloud.points[i].r;
      color.g = vertices_cloud.points[i].g;
      color.b = vertices_cloud.points[i].b;
      color.a = vertices_cloud.points[i].a;
      new_mesh.vertex_colors.push_back(color);
    }
  }

  // Convert polygons
  for (pcl::Vertices polygon : polygon_mesh.polygons) {
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
  // Convert vertices
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  bool color = (mesh_msg.vertex_colors.size() == mesh_msg.vertices.size());
  for (size_t i = 0; i < mesh_msg.vertices.size(); i++) {
    geometry_msgs::Point p = mesh_msg.vertices[i];
    pcl::PointXYZRGBA point;
    point.x = p.x;
    point.y = p.y;
    point.z = p.z;
    if (color) {
      std_msgs::ColorRGBA c = mesh_msg.vertex_colors[i];
      point.r = c.r;
      point.g = c.g;
      point.b = c.b;
      point.a = c.a;
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
                               const pcl::PolygonMesh& mesh2) {
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
    for (size_t j = 0; j < vertices1.points.size(); j++) {
      if (vertices1.points[j].x == vertices2.points[i].x &&
          vertices1.points[j].y == vertices2.points[i].y &&
          vertices1.points[j].z == vertices2.points[i].z) {
        idx = j;
        new_point = false;
        break;
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

pcl::PolygonMesh VoxbloxMeshBlockToPolygonMesh(
    const voxblox_msgs::MeshBlock& mesh_block,
    float block_edge_length) {
  pcl::PolygonMesh new_mesh;
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;

  // Extract mesh block
  size_t vertex_index = 0u;
  // translate vertex data from message to voxblox mesh
  pcl::Vertices triangle;
  for (size_t i = 0; i < mesh_block.x.size(); ++i) {
    // (2*block_size), see mesh_vis.h for the slightly convoluted
    // justification of the 2.
    constexpr float point_conv_factor =
        2.0f / std::numeric_limits<uint16_t>::max();
    const float mesh_x =
        (static_cast<float>(mesh_block.x[i]) * point_conv_factor +
         static_cast<float>(mesh_block.index[0])) *
        block_edge_length;
    const float mesh_y =
        (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
         static_cast<float>(mesh_block.index[1])) *
        block_edge_length;
    const float mesh_z =
        (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
         static_cast<float>(mesh_block.index[2])) *
        block_edge_length;
    const float mesh_r = static_cast<float>(mesh_block.r[i]);
    const float mesh_g = static_cast<float>(mesh_block.g[i]);
    const float mesh_b = static_cast<float>(mesh_block.b[i]);

    // Search if vertex inserted
    size_t vidx;
    bool point_exists = false;
    for (size_t k = 0; k < vertices_cloud.points.size(); k++) {
      if (mesh_x == vertices_cloud.points[k].x &&
          mesh_y == vertices_cloud.points[k].y &&
          mesh_z == vertices_cloud.points[k].z) {
        vidx = k;
        point_exists = true;
        break;
      }
    }
    if (!point_exists) {
      vidx = vertex_index++;
      pcl::PointXYZRGBA point;
      point.x = mesh_x;
      point.y = mesh_y;
      point.z = mesh_z;
      point.r = mesh_r;
      point.g = mesh_g;
      point.b = mesh_b;
      point.a = 1.0;
      vertices_cloud.push_back(point);
    }

    triangle.vertices.push_back(vidx);
    if (triangle.vertices.size() == 3) {
      new_mesh.polygons.push_back(triangle);
      triangle = pcl::Vertices();
    }
  }
  pcl::toPCLPointCloud2(vertices_cloud, new_mesh.cloud);
  return new_mesh;
}

pcl::PolygonMesh VoxbloxToPolygonMesh(
    const voxblox_msgs::Mesh::ConstPtr& voxblox_msg) {
  pcl::PolygonMesh new_mesh;

  // Extract mesh block
  size_t vertex_index = 0u;
  for (const voxblox_msgs::MeshBlock& mesh_block : voxblox_msg->mesh_blocks) {
    pcl::PolygonMesh partial_mesh = VoxbloxMeshBlockToPolygonMesh(
        mesh_block, voxblox_msg->block_edge_length);
    new_mesh = CombineMeshes(new_mesh, partial_mesh);
  }

  return new_mesh;
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
                            const gtsam::Values& values) {
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
      edge.key_from = factor.front();
      edge.key_to = factor.back();
      if (edge.key_to == edge.key_from + 1) {  // check if odom
        edge.type = pose_graph_tools::PoseGraphEdge::ODOM;
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
              factor.get_noiseModel())
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
    pose_graph_tools::PoseGraphNode node;
    node.key = key_list[i];
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

    nodes.push_back(node);
  }

  pose_graph_tools::PoseGraph posegraph;
  posegraph.nodes = nodes;
  posegraph.edges = edges;
  return boost::make_shared<pose_graph_tools::PoseGraph>(posegraph);
}

}  // namespace mesher_mapper