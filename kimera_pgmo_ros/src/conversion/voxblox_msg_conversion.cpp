/**
 * @file   voxblox_utils.cpp
 * @brief
 * @author Yun Chang
 */
#include "kimera_pgmo_ros/conversion/voxblox_msg_conversion.h"

#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>

#include <algorithm>
#include <chrono>
#include <limits>

#include "kimera_pgmo/utils/common_functions.h"

namespace kimera_pgmo::conversions {

pcl::PointXYZRGBA ExtractPoint(const voxblox_msgs::MeshBlock& mesh_block,
                               const float& block_edge_length,
                               const size_t& idx) {
  // (2*block_size), see mesh_vis.h for the slightly convoluted
  // justification of the 2.
  constexpr float point_conv_factor = 2.0f / std::numeric_limits<uint16_t>::max();
  const float mesh_x = (static_cast<float>(mesh_block.x.at(idx)) * point_conv_factor +
                        static_cast<float>(mesh_block.index[0])) *
                       block_edge_length;
  const float mesh_y = (static_cast<float>(mesh_block.y.at(idx)) * point_conv_factor +
                        static_cast<float>(mesh_block.index[1])) *
                       block_edge_length;
  const float mesh_z = (static_cast<float>(mesh_block.z.at(idx)) * point_conv_factor +
                        static_cast<float>(mesh_block.index[2])) *
                       block_edge_length;

  // Create point
  pcl::PointXYZRGBA point;
  point.x = mesh_x;
  point.y = mesh_y;
  point.z = mesh_z;
  point.r = mesh_block.r[idx];
  point.g = mesh_block.g[idx];
  point.b = mesh_block.b[idx];
  point.a = std::numeric_limits<uint8_t>::max();

  return point;
}

pcl::PolygonMesh UpdateMeshFromVoxbloxMeshBlock(
    const voxblox_msgs::MeshBlock& mesh_block,
    const float& block_edge_length,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices,
    std::shared_ptr<std::vector<pcl::Vertices>> triangles,
    const std::vector<size_t>& original_indices,
    std::shared_ptr<std::vector<size_t>> updated_indices,
    std::shared_ptr<std::map<size_t, std::vector<pcl::Vertices>>> adjacent_surfaces) {
  // For speed, assume mesh is incrementally increasing
  if (mesh_block.x.size() <= original_indices.size()) {
    *updated_indices = original_indices;
    return VoxbloxMeshBlockToPolygonMesh(mesh_block, block_edge_length);
  }

  // Mesh corresponding to input mesh block
  pcl::PolygonMesh new_added_mesh;
  pcl::PointCloud<pcl::PointXYZRGBA> new_vertices;
  std::vector<size_t> updated_indices_partial;

  // Extract mesh block
  size_t vertex_index = vertices->points.size();
  size_t vertex_index_new = 0;
  // translate vertex data from message to voxblox mesh
  pcl::Vertices triangle;
  pcl::Vertices triangle_new;  // triangle with indices for partial mesh
  for (size_t i = 0; i < mesh_block.x.size(); ++i) {
    pcl::PointXYZRGBA point = ExtractPoint(mesh_block, block_edge_length, i);
    // Search if vertex inserted
    size_t vidx;
    size_t vidx_new;  // idx for the partial mesh
    bool point_exists = false;
    bool point_exists_in_new = false;  // does point exist in the partial mesh
    for (size_t j : original_indices) {
      if (point.x == vertices->points[j].x && point.y == vertices->points[j].y &&
          point.z == vertices->points[j].z) {
        vidx = j;
        point_exists = true;
        break;
      }
    }

    // Also check the new vertices
    for (size_t j = 0; j < updated_indices->size(); j++) {
      size_t k = updated_indices->at(j);
      if (point.x == vertices->points[k].x && point.y == vertices->points[k].y &&
          point.z == vertices->points[k].z) {
        vidx = k;
        // For partial mesh, the indices should start with 0
        vidx_new = updated_indices_partial.at(j);
        point_exists = true;
        point_exists_in_new = true;
        break;
      }
    }

    // if point exists prior to processing this mesh block
    if (!point_exists) {
      if (!point_exists_in_new) {
        vidx = vertex_index++;
        vertices->push_back(point);
      }
    }
    // if points already added when processing this mesh block
    if (!point_exists_in_new) {
      vidx_new = vertex_index_new++;
      new_vertices.push_back(point);
      updated_indices->push_back(vidx);
      updated_indices_partial.push_back(vidx_new);
    }

    triangle.vertices.push_back(vidx);
    // Keep track of triangle_new separate, since indices different
    triangle_new.vertices.push_back(vidx_new);
    if (triangle.vertices.size() == 3) {
      // Check if surface previously added and insert
      if (!CheckAndUpdateAdjacentSurfaces(triangle, adjacent_surfaces)) {
        triangles->push_back(triangle);
      }
      new_added_mesh.polygons.push_back(triangle_new);
      triangle = pcl::Vertices();
      triangle_new = pcl::Vertices();
    }
  }
  pcl::toPCLPointCloud2(new_vertices, new_added_mesh.cloud);
  return new_added_mesh;
}

pcl::PolygonMesh VoxbloxMeshBlockToPolygonMesh(
    const voxblox_msgs::MeshBlock& mesh_block,
    float block_edge_length) {
  pcl::PolygonMesh new_mesh;
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices_cloud(
      new pcl::PointCloud<pcl::PointXYZRGBA>);
  std::shared_ptr<std::map<size_t, size_t>> msg_vertex_map =
      std::make_shared<std::map<size_t, size_t>>();
  VoxbloxMeshBlockToPolygonMesh(
      mesh_block,
      block_edge_length,
      vertices_cloud,
      msg_vertex_map,
      std::make_shared<std::vector<pcl::Vertices>>(new_mesh.polygons));

  pcl::toPCLPointCloud2(*vertices_cloud, new_mesh.cloud);
  return new_mesh;
}

void VoxbloxMeshBlockToPolygonMesh(
    const voxblox_msgs::MeshBlock& mesh_block,
    float block_edge_length,
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr vertices,
    std::shared_ptr<std::map<size_t, size_t>> msg_vertex_map,
    std::shared_ptr<std::vector<pcl::Vertices>> triangles,
    bool check_duplicates_full) {
  assert(vertices != nullptr);
  assert(triangles != nullptr);
  assert(msg_vertex_map != nullptr);
  // Extract mesh block
  size_t vertex_index = vertices->size();
  size_t first_index_to_check = (check_duplicates_full) ? 0 : vertices->size();
  // translate vertex data from message to voxblox mesh
  pcl::Vertices triangle;
  for (size_t i = 0; i < mesh_block.x.size(); ++i) {
    pcl::PointXYZRGBA point = ExtractPoint(mesh_block, block_edge_length, i);

    // Search if vertex inserted
    size_t vidx;
    bool point_exists = false;
    // Check for duplicates.
    for (size_t k = first_index_to_check; k < vertices->points.size(); k++) {
      if (point.x == vertices->points[k].x && point.y == vertices->points[k].y &&
          point.z == vertices->points[k].z) {
        vidx = k;
        point_exists = true;
        vertices->points[k] = point;
        break;
      }
    }

    if (!point_exists) {
      vidx = vertex_index++;
      vertices->push_back(point);
    }

    triangle.vertices.push_back(vidx);
    msg_vertex_map->insert(std::pair<size_t, size_t>{i, vidx});
    if (triangle.vertices.size() == 3) {
      triangles->push_back(triangle);
      triangle = pcl::Vertices();
    }
  }
  return;
}

pcl::PolygonMesh VoxbloxToPolygonMesh(const voxblox_msgs::Mesh::ConstPtr& voxblox_msg) {
  pcl::PolygonMesh new_mesh;

  // Extract mesh block
  for (const voxblox_msgs::MeshBlock& mesh_block : voxblox_msg->mesh_blocks) {
    pcl::PolygonMesh partial_mesh =
        VoxbloxMeshBlockToPolygonMesh(mesh_block, voxblox_msg->block_edge_length);
    new_mesh = CombineMeshes(new_mesh, partial_mesh);
  }

  return new_mesh;
}

}  // namespace kimera_pgmo::conversions
