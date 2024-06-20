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
using PgmoColor = kimera_pgmo::traits::Color;
using PgmoStamp = kimera_pgmo::traits::Timestamp;
using PgmoLabel = kimera_pgmo::traits::Label;

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
                      std::optional<PgmoColor>*,
                      std::optional<std::uint8_t>*,
                      std::optional<PgmoStamp>*,
                      std::optional<PgmoLabel>*) {
  const auto& p = cloud.at(i);
  return {p.x, p.y, p.z};
}

template <typename T, std::enable_if_t<traits::has_color_v<T>, bool> = true>
PgmoPos pgmoGetVertex(const PointCloud<T>& cloud,
                      size_t i,
                      std::optional<PgmoColor>* color,
                      std::optional<std::uint8_t>* alpha,
                      std::optional<PgmoStamp>*,
                      std::optional<PgmoLabel>*) {
  const auto& p = cloud.at(i);
  if (color) {
    *color = {{p.r, p.g, p.b}};
  }

  if (alpha) {
    *alpha = p.a;
  }

  return {p.x, p.y, p.z};
}

template <typename T, std::enable_if_t<!traits::has_color_v<T>, bool> = true>
void pgmoSetVertex(PointCloud<T>& cloud,
                   size_t i,
                   const PgmoPos& pos,
                   const std::optional<PgmoColor>&,
                   const std::optional<std::uint8_t>&,
                   const std::optional<PgmoStamp>&,
                   const std::optional<PgmoLabel>&) {
  auto& p = cloud.at(i);
  p.x = pos.x();
  p.y = pos.y();
  p.z = pos.z();
}

template <typename T, std::enable_if_t<traits::has_color_v<T>, bool> = true>
void pgmoSetVertex(PointCloud<T>& cloud,
                   size_t i,
                   const PgmoPos& pos,
                   const std::optional<PgmoColor>& color,
                   const std::optional<std::uint8_t>& alpha,
                   const std::optional<PgmoStamp>&,
                   const std::optional<PgmoLabel>&) {
  auto& p = cloud.at(i);
  p.x = pos.x();
  p.y = pos.y();
  p.z = pos.z();

  if (color) {
    const auto& c = *color;
    p.r = c[0];
    p.g = c[1];
    p.b = c[2];
  }

  if (alpha) {
    p.a = *alpha;
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
                          std::optional<traits::Color>* color,
                          std::optional<std::uint8_t>* alpha,
                          std::optional<traits::Timestamp>* stamp,
                          std::optional<traits::Label>* label) {
  if (stamp && i < cloud.stamps.size()) {
    *stamp = cloud.stamps[i];
  }

  return pgmoGetVertex(cloud.points, i, color, alpha, stamp, label);
}

template <typename T>
void pgmoSetVertex(StampedCloud<T>& cloud,
                   size_t i,
                   const traits::Pos& pos,
                   const std::optional<traits::Color>& color,
                   const std::optional<uint8_t>& alpha,
                   const std::optional<traits::Timestamp>& stamp,
                   const std::optional<traits::Label>& label) {
  pgmoSetVertex(cloud.points, i, pos, color, alpha, stamp, label);
  if (stamp) {
    cloud.stamps.at(i) = *stamp;
  }
}

template <typename T>
uint64_t pgmoGetVertexStamp(const ConstStampedCloud<T>& cloud, size_t i) {
  return cloud.stamps.at(i);
}

}  // namespace kimera_pgmo
