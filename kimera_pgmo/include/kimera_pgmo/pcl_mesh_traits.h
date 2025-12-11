/**
 * @file   pcl_mesh_traits.
 * @brief  trait defintions for pcl::PolygonMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <type_traits>

#include "kimera_pgmo/mesh_types.h"

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
void pclToPgmoColor(const T&, PgmoTraits&) {}

template <typename T, std::enable_if_t<!traits::has_color_v<T>, bool> = true>
void pgmoToPclColor(const PgmoTraits&, T&) {}

template <typename T, std::enable_if_t<!traits::has_label_v<T>, bool> = true>
void pclToPgmoLabel(const T&, PgmoTraits&) {}

template <typename T, std::enable_if_t<!traits::has_label_v<T>, bool> = true>
void pgmoToPclLabel(const PgmoTraits&, T&) {}

template <typename T, std::enable_if_t<traits::has_color_v<T>, bool> = true>
void pclToPgmoColor(const T& p, PgmoTraits& traits) {
  traits.color = {{p.r, p.g, p.b, p.a}};
}

template <typename T, std::enable_if_t<traits::has_color_v<T>, bool> = true>
void pgmoToPclColor(const PgmoTraits& traits, T& p) {
  p.r = traits.color[0];
  p.g = traits.color[1];
  p.b = traits.color[2];
  p.a = traits.color[3];
}

template <typename T, std::enable_if_t<traits::has_label_v<T>, bool> = true>
void pclToPgmoLabel(const T& p, PgmoTraits& traits) {
  traits.label = p.label;
}

template <typename T, std::enable_if_t<traits::has_label_v<T>, bool> = true>
void pgmoToPclLabel(const PgmoTraits& traits, T& p) {
  p.label = traits.label;
}

template <typename T>
PgmoPos pgmoGetVertex(const PointCloud<T>& cloud, size_t i, PgmoTraits* traits) {
  const auto& p = cloud.at(i);
  if (traits) {
    pclToPgmoColor<T>(p, *traits);
    pclToPgmoLabel<T>(p, *traits);
  }

  return {p.x, p.y, p.z};
}

template <typename T, std::enable_if_t<!traits::has_color_v<T>, bool> = true>
void pgmoSetVertex(PointCloud<T>& cloud,
                   size_t i,
                   const PgmoPos& pos,
                   const PgmoTraits&) {
  auto& p = cloud.at(i);
  p.x = pos.x();
  p.y = pos.y();
  p.z = pos.z();
}

template <typename T, std::enable_if_t<traits::has_color_v<T>, bool> = true>
void pgmoSetVertex(PointCloud<T>& cloud,
                   size_t i,
                   const PgmoPos& pos,
                   const PgmoTraits& traits) {
  auto& p = cloud.at(i);
  p.x = pos.x();
  p.y = pos.y();
  p.z = pos.z();

  // TODO(nathan) think about field validity
  pgmoToPclColor(traits, p);
  pgmoToPclLabel(traits, p);
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
                   const traits::VertexTraits& traits) {
  pgmoSetVertex(cloud.points, i, pos, traits);
  cloud.stamps.at(i) = traits.stamp;
}

template <typename T>
uint64_t pgmoGetVertexStamp(const ConstStampedCloud<T>& cloud, size_t i) {
  return cloud.stamps.at(i);
}

}  // namespace kimera_pgmo
