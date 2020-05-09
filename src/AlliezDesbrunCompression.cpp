/**
 * @file   AlliezDesbrunCompression.cpp
 * @brief  Simplify and reconstruct meshes Alliez Desbrun 2001
 * @author Yun Chang
 */
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include "mesher_mapper/AlliezDesbrunCompression.h"

namespace mesher_mapper {

AlliezDesbrunCompression::AlliezDesbrunCompression() {}
AlliezDesbrunCompression::~AlliezDesbrunCompression() {}

bool AlliezDesbrunCompression::process() {}

bool AlliezDesbrunCompression::setInputMesh(pcl::PolygonMeshPtr input_mesh) {
  original_mesh_ = *input_mesh;
  // Generate mesh graph
  mesh_graph_.createFromPclMesh(*input_mesh);

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(input_mesh->cloud, cloud);
  // Populate vertex positions
  for (size_t i = 0; i < cloud.points.size(); i++) {
    vertex_positions_[i] =
        Pointxyz(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    vertex_status_[i] = VertexStatus::UNCONQUERED;
    vertex_to_triangle_[i] = std::vector<Polygon<Vertex>>();
  }

  // Then convert polygons to our polygon type
  for (pcl::Vertices polygon : input_mesh->polygons) {
    // Polygons here oriented already using right hand rule
    Polygon<Vertex> new_p(polygon.vertices);
    for (size_t i : polygon.vertices) {
      vertex_to_triangle_.at(i).push_back(new_p);
    }
  }
}

void AlliezDesbrunCompression::decimatingConquest(
    pcl::PolygonMeshPtr output_mesh,
    GraphPtr ouput_graph) {
  // Start queue for gates
  std::vector<Edge> queue;
  // Start with some edge
  Vertex v1 = vertex_status_.begin()->first;
  Vertex v2 = mesh_graph_.getValence(v1)[0];
  Edge e(v1, v2);
  queue.push_back(e);
  // Conquest start
  while (queue.size() > 0) {
    // Pop gate from queue
    Edge gate = queue[0];
    queue.erase(queue.begin());
    // Mark the two vertices of the gate as "conquered"
    vertex_status_[gate.first] = VertexStatus::CONQUERED;
    vertex_status_[gate.second] = VertexStatus::CONQUERED;
    // Get front vertex
    Vertex front_vertex;
    if (getFrontVertex(gate, &front_vertex)) {
      // Check vertex status
      if (vertex_status_[front_vertex] == VertexStatus::UNCONQUERED) {
        Vertices valences = mesh_graph_.getValence(front_vertex);
        if (valences.size() <= 6) {
          // Tag front vertex as to be removed
          vertex_status_[front_vertex] = VertexStatus::REMOVED;
          // Tag valences as conquered
          for (Vertex valence : valences) {
            vertex_status_[valence] = VertexStatus::CONQUERED;
          }
          // Decimate and retriangulate
          //// Get neighboring polygons
          std::vector<Polygon<Vertex>> v1_polygons =
              vertex_to_triangle_[e.first];
          //// Combine and triangulate
          Polygon<Vertex> new_poly = v1_polygons[0];
          for (size_t i = 1; i < v1_polygons.size(); i++) {
            new_poly = new_poly.combine(v1_polygons[i]);
          }
          //// Find index for triangulation
          std::shared_ptr<std::vector<Polygon<Vertex>>> new_triangles =
              std::make_shared<std::vector<Polygon<Vertex>>>();
          if (new_poly.getNumVertices() > 3) {
            std::vector<size_t> tri_ind = findTriangulationIndex(new_poly);
            new_poly.triangulateWithRefidx(tri_ind, new_triangles);
          } else {
            new_triangles->push_back(new_poly);
          }
          // Generate and push new gates
          Vertices new_poly_vertices = new_poly.getVertices();
          for (size_t i = 0; i < new_poly_vertices.size(); i++) {
            // Create edge
            size_t idx_next = (i + 1) % new_poly_vertices.size();
            // Note this is reversed so pointing "out"
            Edge new_gate(new_poly_vertices[idx_next], new_poly_vertices[i]);
            queue.push_back(new_gate);
          }
        }
      }
    }
  }
}

std::vector<size_t> AlliezDesbrunCompression::findTriangulationIndex(
    Polygon<Vertex> p) const {
  // For now choose basic one (ie 0, 2, 4, ...)
  std::vector<size_t> indices;
  for (size_t i = 0; i < p.getNumVertices(); i++) {
    if (i % 2 == 0) {
      indices.push_back(i);
    }
  }
  return indices;
}

bool AlliezDesbrunCompression::getFrontVertex(const Edge& e, Vertex* v) const {
  std::vector<Polygon<Vertex>> v1_polygons = vertex_to_triangle_.at(e.first);
  // Find the polygon that has edge
  for (Polygon<Vertex> p : v1_polygons) {
    Vertices vertices = p.getVertices();
    for (size_t i = 0; i < vertices.size(); i++) {
      if (vertices[i] == e.first) {
        size_t idx_next = (i + 1) % 3;
        if (vertices[idx_next] == e.second) {
          *v = vertices[(idx_next + 1) % 3];
          return true;
        }
      }
    }
  }
  return false;
}

}  // namespace mesher_mapper