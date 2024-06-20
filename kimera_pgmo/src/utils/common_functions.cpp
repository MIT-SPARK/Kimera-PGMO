/**
 * @file   common_functions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */
#include "kimera_pgmo/utils/common_functions.h"

#include <pcl/conversions.h>

#include <limits>

namespace kimera_pgmo {

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

bool CheckAndUpdateAdjacentSurfaces(
    const pcl::Vertices& new_triangle,
    std::shared_ptr<std::map<size_t, std::vector<pcl::Vertices>>> adjacent_surfaces) {
  if (new_triangle.vertices.size() < 3) return false;
  size_t idx0 = new_triangle.vertices.at(0);
  bool exist = false;
  // Check if vertex is new, if new than surface does not exist
  if (adjacent_surfaces->find(idx0) != adjacent_surfaces->end()) {
    // iterate through the adjacent surfaces of the vertex
    for (size_t i = 0; i < adjacent_surfaces->at(idx0).size(); i++) {
      if (adjacent_surfaces->at(idx0)[i].vertices == new_triangle.vertices) {
        exist = true;
        break;
      }
    }
  }

  if (!exist) {
    // Update adjacent surfaces with new surface
    for (size_t idx : new_triangle.vertices) {
      if (adjacent_surfaces->find(idx) == adjacent_surfaces->end()) {
        // Add the new vertex
        adjacent_surfaces->insert(
            std::pair<size_t, std::vector<pcl::Vertices>>(idx, {new_triangle}));
      } else {
        // Add surface
        adjacent_surfaces->at(idx).push_back(new_triangle);
      }
    }
  }

  return exist;
}

}  // namespace kimera_pgmo
