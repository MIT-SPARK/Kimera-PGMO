/**
 * @file   AlliezDesbrunCompression.cpp
 * @brief  Simplify and reconstruct meshes Alliez Desbrun 2001
 * @author Yun Chang
 */
#include <random>

#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <ros/ros.h>

#include "kimera_pgmo/compression/AlliezDesbrunCompression.h"

namespace kimera_pgmo {

// AlliezDesbrunCompression class definitions

AlliezDesbrunCompression::AlliezDesbrunCompression() {}
AlliezDesbrunCompression::~AlliezDesbrunCompression() {}

bool AlliezDesbrunCompression::process() {
  // decimating conquest
  ROS_INFO("decimating conquest");
  pcl::PolygonMeshPtr decimated_mesh(new pcl::PolygonMesh());
  decimatingConquest(6, 0, decimated_mesh);
  base_mesh_ = *decimated_mesh;
  reset(base_mesh_);
  // cleaning conquest
  ROS_INFO("cleaning conquest");
  pcl::PolygonMeshPtr cleaned_mesh(new pcl::PolygonMesh());
  decimatingConquest(3, 0, cleaned_mesh);
  base_mesh_ = *cleaned_mesh;
  reset(base_mesh_);
}

bool AlliezDesbrunCompression::setInputMesh(pcl::PolygonMeshPtr input_mesh) {
  original_mesh_ = *input_mesh;

  reset(*input_mesh);
}

bool AlliezDesbrunCompression::reset(const pcl::PolygonMesh& mesh) {
  // clear
  mesh_graph_ = Graph();
  vertex_positions_.clear();
  vertex_to_triangle_.clear();
  vertex_status_.clear();
  mesh_surfaces_.clear();

  // Generate mesh graph
  mesh_graph_.createFromPclMesh(mesh);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);
  // Populate vertex positions
  for (size_t i = 0; i < cloud.points.size(); i++) {
    vertex_positions_[i] =
        Pointxyz(cloud.points[i].x, cloud.points[i].y, cloud.points[i].z);
    vertex_status_[i] = VertexStatus::UNCONQUERED;
    vertex_to_triangle_[i] = std::vector<Polygon<Vertex>>();
  }
  // Then convert polygons to our polygon type
  for (pcl::Vertices polygon : mesh.polygons) {
    // Polygons here oriented already using right hand rule
    Polygon<Vertex> new_p(polygon.vertices);
    mesh_surfaces_.push_back(new_p);
    for (size_t i : polygon.vertices) {
      vertex_to_triangle_.at(i).push_back(new_p);
    }
  }
}

Edge AlliezDesbrunCompression::findGate() const {
  Vertex v1 = rand() % vertex_status_.size();
  while (mesh_graph_.getValence(v1).size() == 0) {
    v1 = rand() % vertex_status_.size();
  }
  Vertex v12 = mesh_graph_.getValence(v1).at(0);
  return Edge(v1, v12);
}

void AlliezDesbrunCompression::decimatingConquest(
    size_t max_valence,
    size_t min_valence,
    pcl::PolygonMeshPtr output_mesh) {
  // Start queue for gates
  std::vector<Edge> queue;
  // Start with some edge
  Edge e1 = findGate();
  queue.push_back(e1);
  std::vector<Polygon<Vertex>> new_mesh_surfaces;
  // Conquest start
  while (queue.size() > 0) {
    // Pop gate from queue
    Edge gate = queue[0];
    queue.erase(queue.begin());
    // Mark the two vertices of the gate as "conquered"
    if (vertex_status_[gate.first] != VertexStatus::REMOVED)
      vertex_status_[gate.first] = VertexStatus::CONQUERED;
    if (vertex_status_[gate.second] != VertexStatus::REMOVED)
      vertex_status_[gate.second] = VertexStatus::CONQUERED;
    // Get front vertex
    Vertex front_vertex;
    if (getFrontVertex(gate, &front_vertex)) {
      // Check vertex status
      Vertices valences = mesh_graph_.getValence(front_vertex);
      if (vertex_status_[front_vertex] == VertexStatus::UNCONQUERED and
          (valences.size() <= max_valence and valences.size() >= min_valence)) {
        // Tag front vertex as to be removed
        vertex_status_[front_vertex] = VertexStatus::REMOVED;
        // Tag valences as conquered
        for (Vertex valence : valences) {
          if (vertex_status_[valence] != VertexStatus::REMOVED)
            vertex_status_[valence] = VertexStatus::CONQUERED;
        }
        // Decimate and retriangulate
        std::shared_ptr<std::vector<Polygon<Vertex>>> new_triangles =
            std::make_shared<std::vector<Polygon<Vertex>>>();
        //// Get neighboring polygons
        std::vector<Polygon<Vertex>> v1_polygons =
            vertex_to_triangle_[front_vertex];
        //// Combine and triangulate
        Polygon<Vertex> new_poly = v1_polygons[0];
        v1_polygons.erase(v1_polygons.begin());
        size_t count = 0;
        new_poly.print("add");
        while (v1_polygons.size() > 0) {
          Polygon<Vertex> to_be_added = v1_polygons[0];
          to_be_added.print("add");
          v1_polygons.erase(v1_polygons.begin());
          if (!new_poly.combine(to_be_added, &new_poly)) {
            if (v1_polygons.size() > 0 and count < valences.size()) {
              v1_polygons.push_back(to_be_added);
            } else {
              new_triangles->push_back(to_be_added);
            }
            count++;
          } else {
            std::cout << "add successful" << std::endl;
          }
          new_poly.print("new poly");
        }
        new_poly.print("new poly");
        //// Find index for triangulation
        if (new_poly.getNumVertices() > 3) {
          std::vector<size_t> tri_ind = findTriangulationIndex(new_poly);
          new_poly.triangulateWithRefidx(tri_ind, new_triangles);
        } else {
          new_triangles->push_back(new_poly);
        }
        // Track these new mesh surfaces
        for (Polygon<Vertex> t : *new_triangles) {
          new_mesh_surfaces.push_back(t);
          t.print("triangulation");
        }
        // Generate and push new gates
        Vertices new_poly_vertices = new_poly.getVertices();
        for (size_t i = 0; i < new_poly_vertices.size(); i++) {
          // Create edge
          size_t idx_next = (i + 1) % new_poly_vertices.size();
          if (!(new_poly_vertices[i] == gate.first and
                new_poly_vertices[idx_next] == gate.second)) {
            // Note this is reversed so pointing "out"
            Edge new_gate(new_poly_vertices[idx_next], new_poly_vertices[i]);
            queue.push_back(new_gate);
          }
        }
      } else if (vertex_status_[front_vertex] == VertexStatus::UNCONQUERED) {
        // Push the two other edges to gate
        queue.push_back(Edge(gate.first, front_vertex));
        queue.push_back(Edge(front_vertex, gate.second));
      }
    }
  }
  // Make new polygonmesh
  std::map<Vertex, Vertex> old_to_new;  // Mapping vertex number from old to new
  pcl::PointCloud<pcl::PointXYZ> cloud;
  Vertex count = 0;
  for (Vertex i = 0; i < vertex_status_.size(); i++) {
    if (vertex_status_.at(i) != VertexStatus::REMOVED) {
      old_to_new[i] = count;
      cloud.push_back(pcl::PointXYZ(vertex_positions_[i][0],
                                    vertex_positions_[i][1],
                                    vertex_positions_[i][2]));
      count++;
    }
  }

  // Build new PolygonMesh
  pcl::toPCLPointCloud2(cloud, output_mesh->cloud);
  // Add the new mesh surfaces
  for (Polygon<Vertex> p : new_mesh_surfaces) {
    bool null_patch = false;
    for (Vertex v : p.getVertices()) {
      if (vertex_status_.at(v) == VertexStatus::REMOVED) {
        null_patch = true;
        break;
      }
    }
    if (!null_patch) {
      pcl::Vertices new_polygon;
      for (Vertex p_v : p.getVertices()) {
        new_polygon.vertices.push_back(old_to_new[p_v]);
      }
      if (new_polygon.vertices.size() != 3) {
        std::cout << "attempting to push non-triangle surface: ";
        p.print("");
      } else {
        output_mesh->polygons.push_back(new_polygon);
      }
    }
  }
  std::cout << "new surfaces: " << output_mesh->polygons.size() << std::endl;
  // Look through old surfaces add if still relavant
  std::cout << "old surfaces total: " << mesh_surfaces_.size() << std::endl;
  for (Polygon<Vertex> p : mesh_surfaces_) {
    bool null_patch = false;
    for (Vertex v : p.getVertices()) {
      if (vertex_status_.at(v) == VertexStatus::REMOVED) {
        null_patch = true;
        break;
      }
    }
    if (!null_patch) {
      pcl::Vertices new_polygon;
      for (Vertex p_v : p.getVertices()) {
        new_polygon.vertices.push_back(old_to_new[p_v]);
      }
      output_mesh->polygons.push_back(new_polygon);
    }
  }
  std::cout << "total surfaces: " << output_mesh->polygons.size() << std::endl;
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

}  // namespace kimera_pgmo