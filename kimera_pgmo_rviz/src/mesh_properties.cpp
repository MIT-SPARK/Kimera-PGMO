#include "kimera_pgmo_rviz/mesh_properties.h"

#include <QTimer>

#include <rviz_common/properties/int_property.hpp>

#include "kimera_pgmo_rviz/mesh_coloring_property.h"
#include "kimera_pgmo_rviz/mesh_visual.h"

namespace kimera_pgmo {

MeshProperties::MeshProperties(rviz_common::properties::Property* parent) {
  using namespace rviz_common::properties;
  coloring_ = std::make_unique<MeshColoringProperty>(parent);
  max_vertices_ = std::make_unique<IntProperty>(
      "Max Chunk Vertices",
      16384,
      "Maximum vertices per render chunk; triangles are capped at twice this value",
      parent);
  max_vertices_->setMin(3);
  connect(coloring_.get(), &MeshColoringProperty::coloringChanged, this, [this]() {
    ++coloring_revision_;
    scheduleChange();
  });
  connect(
      max_vertices_.get(), &Property::changed, this, &MeshProperties::scheduleChange);
}

MeshProperties::~MeshProperties() = default;

size_t MeshProperties::maxVertices() const { return max_vertices_->getInt(); }

void MeshProperties::scheduleChange() {
  if (change_pending_) {
    return;
  }

  change_pending_ = true;
  QTimer::singleShot(0, this, [this]() {
    change_pending_ = false;
    Q_EMIT changed();
  });
}

void MeshProperties::apply(MeshVisual& visual) const {
  if (visual.coloringRevision() != coloring_revision_) {
    visual.setColoring(coloring_->createColoring(), coloring_revision_);
  }

  visual.setMaxVertices(maxVertices());
}

}  // namespace kimera_pgmo
