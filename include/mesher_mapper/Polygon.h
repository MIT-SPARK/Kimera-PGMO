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
    // Get the common edges
    std::vector<std::pair<size_t, size_t>> common_edges_in_self;
    std::vector<std::pair<size_t, size_t>> common_edges_in_p;
    for (size_t i = 0; i < vertices_.size(); i++) {
      std::pair<T, T> edge(vertices_[i], vertices_[(i + 1) % vertices_.size()]);
      for (size_t j = 0; j < p_vertices.size(); j++) {
        // Note with correct orientation, the common edge would be flipped in
        // direction
        if (edge.second == p_vertices[j] and
            edge.first == p_vertices[(j + 1) % p_vertices.size()]) {
          common_edges_in_self.push_back(
              std::pair<size_t, size_t>(i, (i + 1) % vertices_.size()));
          common_edges_in_p.push_back(
              std::pair<size_t, size_t>(j, (j + 1) % p_vertices.size()));
          break;
        }
      }
    }
    // First add from self polygon until reaching a common edge
    std::vector<T> new_vertices;
    T first_intersection;
    size_t pushed_idx = 0;
    for (size_t i = 0; i < vertices_.size(); i++) {
      // check if in common edges
      std::pair<size_t, size_t> e(i, (i + 1) % vertices_.size());
      std::vector<std::pair<size_t, size_t>>::iterator it;
      it = std::find(
          common_edges_in_self.begin(), common_edges_in_self.end(), e);
      if (it != common_edges_in_self.end()) {
        first_intersection = vertices_[i];
        break;
      } else {
        new_vertices.push_back(vertices_[i]);
        pushed_idx = i;
      }
    }

    // Find position of first vertex in common edge reached
    typename std::vector<T>::iterator it_p;
    it_p = std::find(p_vertices.begin(), p_vertices.end(), first_intersection);
    std::rotate(p_vertices.begin(), it_p, p_vertices.end());
    size_t shift = it_p - p_vertices.begin();

    T second_intersection;
    // Add from p until reach common edge
    for (size_t i = 0; i < p_vertices.size(); i++) {
      // check if in common edges
      std::pair<size_t, size_t> e((i + shift) % p_vertices.size(),
                                  (i + shift + 1) % p_vertices.size());
      std::vector<std::pair<size_t, size_t>>::iterator it;
      it = std::find(common_edges_in_p.begin(), common_edges_in_p.end(), e);
      if (it != common_edges_in_p.end()) {
        second_intersection = p_vertices[i];
        break;
      } else {
        new_vertices.push_back(p_vertices[i]);
      }
    }

    // Back to self and add till end
    // Find position of first vertex in common edge reached
    typename std::vector<T>::const_iterator it_s;
    it_s = std::find(vertices_.begin(), vertices_.end(), second_intersection);
    size_t intersection_idx = it_s - vertices_.begin();
    // Add from p until reach common edge
    if (intersection_idx > pushed_idx) {
      for (size_t i = intersection_idx; i < vertices_.size(); i++) {
        // check if in common edges
        std::pair<size_t, size_t> e(i, (i + 1) % vertices_.size());
        std::vector<std::pair<size_t, size_t>>::iterator it;
        it = std::find(
            common_edges_in_self.begin(), common_edges_in_self.end(), e);
        if (it != common_edges_in_self.end()) {
          break;
        } else {
          new_vertices.push_back(vertices_[i]);
        }
      }
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
    if (ref_indices.size() == 3) {
      std::vector<T> last_triangle;
      for (size_t idx : ref_indices) {
        last_triangle.push_back(vertices_copy[idx]);
      }
      triangles_ptr->push_back(Polygon(last_triangle));
    }

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
      if (vertices_.size() % 2 == 0 and i == vertices_copy.size() - 1) {
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