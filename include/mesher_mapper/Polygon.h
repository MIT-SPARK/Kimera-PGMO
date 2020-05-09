/**
 * @file   Polygon.h
 * @brief  Definitions for the Polygon class
 * @author Yun Chang
 */
#pragma once

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

namespace mesher_mapper {
template <class T>
class Polygon {
 public:
  inline size_t getNumVertices() const { return vertices_.size(); }
  inline std::vector<T> getVertices() const { return vertices_; }

 private:
  std::vector<T> vertices_;  // vertices such that polygon
                             // always on left
 public:
  Polygon(std::vector<T> vertices) { vertices_ = vertices; }
  ~Polygon() {}

  bool equal(const Polygon& p) const {
    if (getNumVertices() != p.getNumVertices()) return false;
    for (size_t i = 0; i < getNumVertices(); i++) {
      std::vector<T> p_vertices = p.getVertices();
      std::rotate(p_vertices.begin(), p_vertices.begin() + i, p_vertices.end());
      if (p_vertices == vertices_) return true;
    }
    return false;
  }

  Polygon combine(const Polygon& p) const {
    // First find the common vertices
    // Asuming that the polygon has no holes
    std::vector<T> p_vertices = p.getVertices();
    // Get the indices of the common vertices
    std::vector<std::pair<size_t, size_t>> common_vertices;
    for (size_t i = 0; i < vertices_.size(); i++) {
      for (size_t j = 0; j < p_vertices.size(); j++) {
        if (vertices_[i] == p_vertices[j]) {
          common_vertices.push_back(std::pair<size_t, size_t>(i, j));
        }
      }
    }
    // First remove the common vertices (if no hole should be consecutive
    std::vector<T> new_vertices;
    size_t new_polygon_size =
        vertices_.size() + p_vertices.size() - common_vertices.size();
    new_vertices.reserve(new_polygon_size);
    for (size_t i = 0; i < common_vertices[0].first; i++) {
      std::cout << i << " " << vertices_[i] << std::endl;
      new_vertices.push_back(vertices_[i]);
    }
    std::rotate(p_vertices.begin(),
                p_vertices.begin() + common_vertices[0].second,
                p_vertices.end());
    // here bascially we are doing (a + b)
    // new vertices: a1 a2 a3 b2 b3 b4 b5 a5 a6
    // if a4 == b2 and a5 == b6
    size_t k = common_vertices.size() - 1;
    size_t last_vertex_to_add =
        common_vertices[k].second - common_vertices[0].second;
    for (size_t i = 0; i < last_vertex_to_add; i++) {
      new_vertices.push_back(p_vertices[i]);
    }
    for (size_t i = common_vertices[k].first; i < vertices_.size(); i++) {
      new_vertices.push_back(vertices_[i]);
    }

    Polygon new_polygon(new_vertices);
    return new_polygon;
  }

  void triangulate(std::shared_ptr<std::vector<Polygon>> trianlges_ptr) {
    // TODO
  }

  void triangulateWithRefidx(
      std::vector<size_t> ref_indices,
      std::shared_ptr<std::vector<Polygon>> triangles_ptr) {
    if (ref_indices.size() > 3) {
      std::cout << "triangulateWithRefidx: For now only supports polygons with "
                   "less than 7 vertices"
                << std::endl;
    }
    // sort vector from low to high indices
    std::sort(ref_indices.begin(), ref_indices.end());
    // Make it so that the first element in vertices is marked
    size_t first_index = ref_indices[0];
    std::vector<T> vertices_copy = vertices_;
    std::rotate(vertices_copy.begin(),
                vertices_copy.begin() + first_index,
                vertices_copy.end());
    for (size_t i = 0; i < ref_indices.size(); i++) {
      ref_indices[i] = ref_indices[i] - first_index;
    }

    // Push back last triangle
    std::vector<T> last_triangle;
    for (size_t idx : ref_indices) {
      last_triangle.push_back(vertices_copy[idx]);
    }
    triangles_ptr->push_back(Polygon(last_triangle));

    std::vector<T> new_triangle{vertices_copy[0]};
    ref_indices.erase(ref_indices.begin());
    for (size_t i = 1; i < vertices_copy.size() + 1; i++) {
      // Push new vertex to new triangle
      new_triangle.push_back(vertices_copy[i]);
      if (i == ref_indices[0]) {
        // Push and reset triangle
        if (new_triangle.size() == 3) {
          triangles_ptr->push_back(Polygon(new_triangle));
          new_triangle = std::vector<T>{vertices_[i]};
        }
        ref_indices.erase(ref_indices.begin());
      }
      // For very last idx need to cycle back to first 
      if (i == vertices_copy.size() - 1) {
        new_triangle.push_back(vertices_copy[0]);
        triangles_ptr->push_back(Polygon(new_triangle));
      }
    }
  }

  void print(std::string heading = "") const {
    // Print polygon
    std::cout << heading << " : ";
    for (T i : vertices_) {
      std::cout << i << " ";
    }
    std::cout << std::endl;
  }
};
}  // namespace mesher_mapper