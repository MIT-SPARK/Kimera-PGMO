/**
 * @file   mesh_traits.h
 * @brief  adl-based mesh getters/setters and traits
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <pcl/Vertices.h>

#include <Eigen/Dense>
#include <array>
#include <cstdint>
#include <optional>
#include <type_traits>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

struct SimpleMesh {
  std::vector<traits::Pos> points;
  std::vector<traits::Timestamp> stamps;
  std::vector<traits::Face> faces;
};

size_t pgmoNumVertices(const SimpleMesh& mesh);

void pgmoResizeVertices(SimpleMesh& mesh, size_t size);

traits::Pos pgmoGetVertex(const SimpleMesh& mesh,
                          size_t i,
                          traits::VertexTraits* traits);

void pgmoSetVertex(SimpleMesh& mesh,
                   size_t i,
                   const traits::Pos& pos,
                   const traits::VertexTraits& traits);
size_t pgmoNumFaces(const SimpleMesh& mesh);

void pgmoResizeFaces(SimpleMesh& mesh, size_t size);

traits::Face pgmoGetFace(const SimpleMesh& mesh, size_t i);

void pgmoSetFace(SimpleMesh& mesh, size_t i, const traits::Face& face);

uint64_t pgmoGetVertexStamp(const SimpleMesh& mesh, size_t i);

size_t pgmoNumFaces(const std::vector<pcl::Vertices>& faces);

traits::Face pgmoGetFace(const std::vector<pcl::Vertices>& faces, size_t i);

void pgmoResizeFaces(std::vector<pcl::Vertices>& faces, size_t size);

void pgmoSetFace(std::vector<pcl::Vertices>& faces, size_t i, const traits::Face& face);

namespace traits {
namespace detail {

// see https://www.open-std.org/jtc1/sc22/wg21/docs/papers/2015/n4381.html

// ODR workaround
template <class T>
constexpr T static_const{};

struct vertex_size_fn {
  template <typename T>
  constexpr auto operator()(const T& v) const -> decltype(pgmoNumVertices(v)) {
    return pgmoNumVertices(v);
  }
};

struct vertex_resize_fn {
  template <typename T>
  constexpr auto operator()(T& vertices, size_t size) const
      -> decltype(pgmoResizeVertices(vertices, size)) {
    return pgmoResizeVertices(vertices, size);
  }
};

struct vertex_get_fn {
  template <typename T>
  constexpr auto operator()(const T& v, size_t i, traits::VertexTraits* t) const
      -> decltype(pgmoGetVertex(v, i, t)) {
    return pgmoGetVertex(v, i, t);
  }
};

struct vertex_set_fn {
  template <typename T>
  constexpr auto operator()(T& v, size_t i, const Pos& p, const traits::VertexTraits& t)
      const -> decltype(pgmoSetVertex(v, i, p, t)) {
    return pgmoSetVertex(v, i, p, t);
  }
};

struct face_size_fn {
  template <typename T>
  constexpr auto operator()(const T& f) const -> decltype(pgmoNumFaces(f)) {
    return pgmoNumFaces(f);
  }
};

struct face_resize_fn {
  template <typename T>
  constexpr auto operator()(T& f, size_t s) const -> decltype(pgmoResizeFaces(f, s)) {
    return pgmoResizeFaces(f, s);
  }
};
struct face_get_fn {
  template <typename T>
  constexpr auto operator()(const T& f, size_t i) const -> decltype(pgmoGetFace(f, i)) {
    return pgmoGetFace(f, i);
  }
};

struct face_set_fn {
  template <typename T>
  constexpr auto operator()(T& f,
                            size_t i,
                            const Face& t) const -> decltype(pgmoSetFace(f, i, t)) {
    return pgmoSetFace(f, i, t);
  }
};

struct vertex_stamp_fn {
  template <typename T>
  constexpr auto operator()(const T& v,
                            size_t i) const -> decltype(pgmoGetVertexStamp(v, i)) {
    return pgmoGetVertexStamp(v, i);
  }
};

}  // namespace detail

namespace {

constexpr const auto& pgmoNumVertices = detail::static_const<detail::vertex_size_fn>;
constexpr const auto& pgmoResizeVertices =
    detail::static_const<detail::vertex_resize_fn>;
constexpr const auto& pgmoGetVertex = detail::static_const<detail::vertex_get_fn>;
constexpr const auto& pgmoSetVertex = detail::static_const<detail::vertex_set_fn>;

constexpr const auto& pgmoNumFaces = detail::static_const<detail::face_size_fn>;
constexpr const auto& pgmoResizeFaces = detail::static_const<detail::face_resize_fn>;
constexpr const auto& pgmoGetFace = detail::static_const<detail::face_get_fn>;
constexpr const auto& pgmoSetFace = detail::static_const<detail::face_set_fn>;

constexpr const auto& pgmoGetVertexStamp =
    detail::static_const<detail::vertex_stamp_fn>;

}  // namespace

// vertices

template <typename T>
size_t num_vertices(const T& vertices) {
  return ::kimera_pgmo::traits::pgmoNumVertices(vertices);
}

template <typename T>
void resize_vertices(T& vertices, size_t size) {
  ::kimera_pgmo::traits::pgmoResizeVertices(vertices, size);
}

template <typename T>
Pos get_vertex(const T& vertices, size_t i, traits::VertexTraits* traits = nullptr) {
  return ::kimera_pgmo::traits::pgmoGetVertex(vertices, i, traits);
}

template <typename T>
void set_vertex(T& vertices,
                size_t i,
                const Pos& pos,
                const traits::VertexTraits& traits = {}) {
  ::kimera_pgmo::traits::pgmoSetVertex(vertices, i, pos, traits);
}

// faces

template <typename T>
size_t num_faces(const T& faces) {
  return ::kimera_pgmo::traits::pgmoNumFaces(faces);
}

template <typename T>
void resize_faces(T& faces, size_t size) {
  ::kimera_pgmo::traits::pgmoResizeFaces(faces, size);
}

template <typename T>
Face get_face(const T& faces, size_t i) {
  return ::kimera_pgmo::traits::pgmoGetFace(faces, i);
}

template <typename T>
void set_face(T& faces, size_t i, const Face& face) {
  ::kimera_pgmo::traits::pgmoSetFace(faces, i, face);
}

using StampGetter = decltype(::kimera_pgmo::traits::pgmoGetVertexStamp);

template <typename T>
using has_get_stamp = std::is_invocable<StampGetter, const T&, size_t>;

template <typename T, std::enable_if_t<!has_get_stamp<T>::value, bool> = true>
Timestamp get_timestamp(const T&, size_t) {
  // methods that rely on the timestamp being valid should dispatch based on
  // has_get_stamp (to avoid the 0 default value)
  return 0;
}

template <typename T, std::enable_if_t<has_get_stamp<T>::value, bool> = true>
Timestamp get_timestamp(const T& vertices, size_t i) {
  return ::kimera_pgmo::traits::pgmoGetVertexStamp(vertices, i);
}

}  // namespace traits
}  // namespace kimera_pgmo
