/**
 * @file   pcl_mesh_traits.
 * @brief  trait defintions for pcl::PolygonMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <pcl/Vertices.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <type_traits>

#include "kimera_pgmo/mesh_traits.h"  // IWYU pragma: keep
#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo::traits {

template <>
size_t num_faces(const std::vector<pcl::Vertices>& faces);

template <>
Face get_face(const std::vector<pcl::Vertices>& faces, size_t i);

template <>
void resize_faces(std::vector<pcl::Vertices>& faces, size_t size);

template <>
void set_face(std::vector<pcl::Vertices>& faces, size_t i, const Face& face);

}  // namespace kimera_pgmo::traits

namespace pcl {

using PgmoPos = kimera_pgmo::traits::Pos;
using PgmoTraits = kimera_pgmo::traits::VertexTraits;
using PgmoProps = kimera_pgmo::traits::VertexProperties;

template <typename T>
size_t pgmoNumVertices(const PointCloud<T>& cloud) {
  return cloud.size();
}

template <typename T>
void pgmoResizeVertices(PointCloud<T>& cloud, size_t size) {
  cloud.resize(size);
}

template <typename T>
PgmoProps pgmoGetVertexProperties(const PointCloud<T>& cloud) {
  return {traits::has_color_v<T>, false, traits::has_label_v<T>, false};
}

template <typename T, std::enable_if_t<!traits::has_color_v<T>, bool> = true>
void setPgmoColorFromPcl(const T&, PgmoTraits&) {}

template <typename T, std::enable_if_t<!traits::has_color_v<T>, bool> = true>
void setPclColorFromPgmo(const PgmoTraits&, T&) {}

template <typename T, std::enable_if_t<!traits::has_label_v<T>, bool> = true>
void setPgmoLabelFromPcl(const T&, PgmoTraits&) {}

template <typename T, std::enable_if_t<!traits::has_label_v<T>, bool> = true>
void setPclLabelFromPgmo(const PgmoTraits&, T&) {}

template <typename T, std::enable_if_t<traits::has_color_v<T>, bool> = true>
void setPgmoColorFromPcl(const T& p, PgmoTraits& traits) {
  traits.color = {{p.r, p.g, p.b, p.a}};
}

template <typename T, std::enable_if_t<traits::has_color_v<T>, bool> = true>
void setPclColorFromPgmo(const PgmoTraits& traits, T& p) {
  if (traits.properties.has_color) {
    p.r = traits.color[0];
    p.g = traits.color[1];
    p.b = traits.color[2];
    p.a = traits.color[3];
  }
}

template <typename T, std::enable_if_t<traits::has_label_v<T>, bool> = true>
void setPgmoLabelFromPcl(const T& p, PgmoTraits& traits) {
  traits.label = p.label;
}

template <typename T, std::enable_if_t<traits::has_label_v<T>, bool> = true>
void setPclLabelFromPgmo(const PgmoTraits& traits, T& p) {
  if (traits.properties.has_label) {
    p.label = traits.label;
  }
}

template <typename T>
PgmoPos pgmoGetVertex(const PointCloud<T>& cloud, size_t i, PgmoTraits* traits) {
  const auto& p = cloud.at(i);
  if (traits) {
    setPgmoColorFromPcl<T>(p, *traits);
    setPgmoLabelFromPcl<T>(p, *traits);
  }

  return {p.x, p.y, p.z};
}

template <typename T>
void pgmoSetVertex(PointCloud<T>& cloud,
                   size_t i,
                   const PgmoPos& pos,
                   const PgmoTraits* traits) {
  auto& p = cloud.at(i);
  p.x = pos.x();
  p.y = pos.y();
  p.z = pos.z();
  if (traits) {
    setPclColorFromPgmo(*traits, p);
    setPclLabelFromPgmo(*traits, p);
  }
}

}  // namespace pcl

namespace kimera_pgmo {

template <typename T>
struct StampedCloud {
  StampedCloud(pcl::PointCloud<T>& points, std::vector<uint64_t>& stamps)
      : points(points), stamps(stamps) {}

  pcl::PointCloud<T>& points;
  std::vector<uint64_t>& stamps;
};

template <typename T>
struct ConstStampedCloud {
  ConstStampedCloud(const pcl::PointCloud<T>& points,
                    const std::vector<uint64_t>& stamps)
      : points(points), stamps(stamps) {}

  const pcl::PointCloud<T>& points;
  const std::vector<uint64_t>& stamps;
};

template <typename T>
size_t pgmoNumVertices(const StampedCloud<T>& cloud) {
  return cloud.points.size();
}

template <typename T>
size_t pgmoNumVertices(const ConstStampedCloud<T>& cloud) {
  return cloud.points.size();
}

template <typename T>
void pgmoResizeVertices(StampedCloud<T>& cloud, size_t size) {
  cloud.points.resize(size);
  cloud.stamps.resize(size);
}

template <typename T>
traits::VertexProperties pgmoGetVertexProperties(const StampedCloud<T>& cloud) {
  auto traits = pgmoGetVertexProperties(cloud.points);
  traits.has_stamp = true;
  return traits;
}

template <typename T>
traits::VertexProperties pgmoGetVertexProperties(const ConstStampedCloud<T>& cloud) {
  auto traits = pgmoGetVertexProperties(cloud.points);
  traits.has_stamp = true;
  return traits;
}

template <typename T>
traits::Pos pgmoGetVertex(const ConstStampedCloud<T>& cloud,
                          size_t i,
                          traits::VertexTraits* traits) {
  if (traits) {
    traits->stamp = cloud.stamps.at(i);
  }

  return pgmoGetVertex(cloud.points, i, traits);
}

template <typename T>
void pgmoSetVertex(StampedCloud<T>& cloud,
                   size_t i,
                   const traits::Pos& pos,
                   const traits::VertexTraits* traits) {
  pgmoSetVertex(cloud.points, i, pos, traits);
  if (traits and traits->properties.has_stamp) {
    cloud.stamps.at(i) = traits->stamp;
  }
}

template <typename T>
uint64_t pgmoGetVertexStamp(const ConstStampedCloud<T>& cloud, size_t i) {
  return cloud.stamps.at(i);
}

}  // namespace kimera_pgmo
