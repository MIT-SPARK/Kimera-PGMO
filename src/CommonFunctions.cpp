/**
 * @file   CommonFunctions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#include <geometry_msgs/Point.h>
#include <mesh_msgs/TriangleIndices.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>
#include <voxblox_msgs/MeshBlock.h>

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

  std::shared_ptr<tinyply::PlyData> vertices, faces;

  // The header information can be used to programmatically extract properties
  // on elements known to exist in the header prior to reading the data. For
  // brevity of this sample, properties like vertex position are hard-coded:
  try {
    vertices = file.request_properties_from_element("vertex", {"x", "y", "z"});
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

  // Extract read data
  const size_t numVerticesBytes = vertices->buffer.size_bytes();
  std::vector<float[3]> verts(vertices->count);
  std::memcpy(verts.data(), vertices->buffer.get(), numVerticesBytes);
  // Convert to pointcloud
  pcl::PointCloud<pcl::PointXYZ> vertices_cloud;
  for (auto v : verts) {
    vertices_cloud.points.push_back(pcl::PointXYZ(v[0], v[1], v[2]));
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
  std::vector<uint3> triangles;
  // Convert to polygon mesh type
  for (pcl::Vertices tri : mesh.polygons) {
    uint3 triangle;
    triangles.push_back({tri.vertices[0], tri.vertices[1], tri.vertices[2]});
  }
  // Get point cloud
  pcl::PointCloud<pcl::PointXYZ> vertices_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices_cloud);

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
      vertices_cloud.points.size(),
      reinterpret_cast<uint8_t*>(vertices_cloud.points.data()),
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
  pcl::PointCloud<pcl::PointXYZ> vertices_cloud;
  pcl::fromPCLPointCloud2(polygon_mesh.cloud, vertices_cloud);

  mesh_msgs::TriangleMesh new_mesh;
  // Convert vertices
  for (size_t i = 0; i < vertices_cloud.points.size(); i++) {
    geometry_msgs::Point p;
    p.x = vertices_cloud.points[i].x;
    p.y = vertices_cloud.points[i].y;
    p.z = vertices_cloud.points[i].z;
    new_mesh.vertices.push_back(p);
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
  pcl::PointCloud<pcl::PointXYZ> vertices_cloud;
  for (size_t i = 0; i < mesh_msg.vertices.size(); i++) {
    geometry_msgs::Point p = mesh_msg.vertices[i];
    pcl::PointXYZ point(p.x, p.y, p.z);
    vertices_cloud.push_back(point);
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

pcl::PolygonMesh VoxbloxGetLastMeshBlock(
    const voxblox_msgs::Mesh::ConstPtr& voxblox_msg) {
  pcl::PolygonMesh new_mesh;
  pcl::PointCloud<pcl::PointXYZ> vertices_cloud;

  // Extract mesh block
  size_t vertex_index = 0u;
  for (const voxblox_msgs::MeshBlock& mesh_block : voxblox_msg->mesh_blocks) {
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
          voxblox_msg->block_edge_length;
      const float mesh_y =
          (static_cast<float>(mesh_block.y[i]) * point_conv_factor +
           static_cast<float>(mesh_block.index[1])) *
          voxblox_msg->block_edge_length;
      const float mesh_z =
          (static_cast<float>(mesh_block.z[i]) * point_conv_factor +
           static_cast<float>(mesh_block.index[2])) *
          voxblox_msg->block_edge_length;

      pcl::PointXYZ point(mesh_x, mesh_y, mesh_z);
      vertices_cloud.push_back(point);

      triangle.vertices.push_back(vertex_index++);
      if (i % 3 == 0) {
        if (i != 0) {
          new_mesh.polygons.push_back(triangle);
          triangle = pcl::Vertices();
        }
      }
    }
  }
  pcl::toPCLPointCloud2(vertices_cloud, new_mesh.cloud);
  return new_mesh;
}

}  // namespace mesher_mapper