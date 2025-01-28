/**
 * @file   pcl_mesh_traits.
 * @brief  trait defintions for pcl::PolygonMesh
 * @author Yun Chang
 * @author Nathan Hughes
 */
#pragma once
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "kimera_pgmo/mesh_types.h"

namespace pcl {

using PgmoPos = kimera_pgmo::traits::Pos;
using PgmoTraits = kimera_pgmo::traits::VertexTraits;


template <typename T>
size_t pgmoNumVertices(const PointCloud<T>& cloud) {
  return cloud.size();
}

template <typename T>
void pgmoResizeVertices(PointCloud<T>& cloud, size_t size) {
  cloud.resize(size);
}

template <typename T, std::enable_if_t<!traits::has_color_v<T>, bool> = true>
PgmoPos pgmoGetVertex(const PointCloud<T>& cloud,
                      size_t i,
                      PgmoTraits*) {
  const auto& p = cloud.at(i);
  return {p.x, p.y, p.z};
}

template <typename T, std::enable_if_t<traits::has_color_v<T>, bool> = true>
PgmoPos pgmoGetVertex(const PointCloud<T>& cloud,
                      size_t i,
                      PgmoTraits* traits) {
  const auto& p = cloud.at(i);
  if (traits) {
    traits->color = {{p.r, p.g, p.b, p.a}};
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

  if (traits.color) {
    const auto& c = *traits.color;
    p.r = c[0];
    p.g = c[1];
    p.b = c[2];
    p.a = c[3];
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
traits::Pos pgmoGetVertex(const ConstStampedCloud<T>& cloud,
                          size_t i,
                          traits::VertexTraits* traits) {
  if (traits && i < cloud.stamps.size()) {
    traits->stamp = cloud.stamps[i];
  }

  return pgmoGetVertex(cloud.points, i, traits);
}

template <typename T>
void pgmoSetVertex(StampedCloud<T>& cloud,
                   size_t i,
                   const traits::Pos& pos,
                   const traits::VertexTraits& traits) {
  pgmoSetVertex(cloud.points, i, pos, traits);
  if (traits.stamp) {
    cloud.stamps.at(i) = *traits.stamp;
  }
}

template <typename T>
uint64_t pgmoGetVertexStamp(const ConstStampedCloud<T>& cloud, size_t i) {
  return cloud.stamps.at(i);
}

}  // namespace kimera_pgmo
