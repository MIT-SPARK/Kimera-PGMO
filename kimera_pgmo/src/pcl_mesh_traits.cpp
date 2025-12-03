#include "kimera_pgmo/pcl_mesh_traits.h"

namespace kimera_pgmo {

LabeledCloud::LabeledCloud(Cloud& points, std::vector<traits::Label>& labels)
    : points(points), labels(labels) {}

LabeledStampedCloud::LabeledStampedCloud(Cloud& points,
                                         std::vector<traits::Timestamp>& stamps,
                                         std::vector<traits::Label>& labels)
    : LabeledCloud(points, labels), stamps(stamps) {}

size_t pgmoNumVertices(const LabeledCloud& cloud) { return cloud.points.size(); }

void pgmoResizeVertices(LabeledCloud& cloud, size_t size) {
  cloud.points.resize(size);
  cloud.labels.resize(size);
}

size_t pgmoNumVertices(const LabeledStampedCloud& cloud) { return cloud.points.size(); }

void pgmoResizeVertices(LabeledStampedCloud& cloud, size_t size) {
  pgmoResizeVertices(static_cast<LabeledCloud&>(cloud), size);
  cloud.stamps.resize(size);
}

void pgmoSetVertex(LabeledCloud& cloud,
                   size_t i,
                   const traits::Pos& pos,
                   const traits::VertexTraits& traits) {
  pgmoSetVertex(cloud.points, i, pos, traits);
  if (traits.label) {
    cloud.labels.at(i) = *traits.label;
  }
}

void pgmoSetVertex(LabeledStampedCloud& cloud,
                   size_t i,
                   const traits::Pos& pos,
                   const traits::VertexTraits& traits) {
  pgmoSetVertex(static_cast<LabeledCloud&>(cloud), i, pos, traits);
  if (traits.stamp) {
    cloud.stamps.at(i) = *traits.stamp;
  }
}

}  // namespace kimera_pgmo
