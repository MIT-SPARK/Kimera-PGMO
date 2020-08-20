/**
 * @file   VoxbloxUtils.cpp
 * @brief  Some utility functions in working with voxblox msg types
 * @author Yun Chang
 */
#include <algorithm>
#include <chrono>
#include <limits>

#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_msgs/PolygonMesh.h>
#include <voxblox_msgs/Mesh.h>
#include <voxblox_msgs/MeshBlock.h>
#include <pcl/impl/point_types.hpp>

#include "kimera_pgmo/utils/VoxbloxUtils.h"

pcl::PolygonMesh UpdateMeshFromVoxbloxMeshBlock(
    const pcl::PolygonMesh& mesh,
    const voxblox_msgs::MeshBlock& mesh_block,
    const float& block_edge_length,
    const std::vector<size_t>& original_indices,
    std::vector<size_t>* updated_indices) {
  // For speed, assume mesh is incrementally increasing
  if (mesh_block.x.size() <= original_indices.size()) {
    *updated_indices = original_indices;
    return mesh;
  }

  pcl::PointCloud<pcl::PointXYZRGBA> vertices1;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices1);

  pcl::PolygonMesh out_mesh;
  out_mesh.polygons = mesh.polygons;

  // Extract mesh block
  size_t vertex_index = vertices1.points.size();
  // translate vertex data from message to voxblox mesh
  pcl::Vertices triangle;
  bool new_triangle = false;
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

    // Create point
    pcl::PointXYZRGBA point;
    point.x = mesh_x;
    point.y = mesh_y;
    point.z = mesh_z;
    point.r = mesh_block.r[i];
    point.g = mesh_block.g[i];
    point.b = mesh_block.b[i];
    point.a = std::numeric_limits<uint8_t>::max();

    // Search if vertex inserted
    size_t vidx;
    bool point_exists = false;
    for (size_t j : original_indices) {
      if (mesh_x == vertices1.points[j].x && mesh_y == vertices1.points[j].y &&
          mesh_z == vertices1.points[j].z) {
        vidx = j;
        point_exists = true;
        break;
      }
    }

    if (!point_exists) {
      vidx = vertex_index++;
      vertices1.push_back(point);
      new_triangle = true;
    }

    updated_indices->push_back(vidx);

    triangle.vertices.push_back(vidx);
    if (triangle.vertices.size() == 3) {
      if (new_triangle) {
        out_mesh.polygons.push_back(triangle);
      }
      triangle = pcl::Vertices();
      new_triangle = false;
    }
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

    // Create point
    pcl::PointXYZRGBA point;
    point.x = mesh_x;
    point.y = mesh_y;
    point.z = mesh_z;
    point.r = mesh_block.r[i];
    point.g = mesh_block.g[i];
    point.b = mesh_block.b[i];
    point.a = std::numeric_limits<uint8_t>::max();

    // Search if vertex inserted
    size_t vidx;
    bool point_exists = false;
    // TODO(Yun) check if this needed. discard to save time
    for (size_t k = 0; k < vertices_cloud.points.size(); k++) {
      if (mesh_x == vertices_cloud.points[k].x &&
          mesh_y == vertices_cloud.points[k].y &&
          mesh_z == vertices_cloud.points[k].z) {
        vidx = k;
        point_exists = true;
        vertices_cloud.points[k] = point;
        break;
      }
    }

    if (!point_exists) {
      vidx = vertex_index++;
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