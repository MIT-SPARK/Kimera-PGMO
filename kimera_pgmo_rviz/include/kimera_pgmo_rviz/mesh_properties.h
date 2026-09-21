#pragma once
#include <QObject>
#include <memory>

namespace rviz_common::properties {
class Property;
class IntProperty;
}  // namespace rviz_common::properties

namespace kimera_pgmo {

class MeshVisual;
class MeshColoringProperty;

class MeshProperties : public QObject {
  Q_OBJECT
 public:
  explicit MeshProperties(rviz_common::properties::Property* parent);
  ~MeshProperties() override;
  size_t maxVertices() const;
  size_t coloringRevision() const { return coloring_revision_; }
  void apply(MeshVisual& visual) const;

 Q_SIGNALS:
  void changed();

 private:
  void scheduleChange();

  std::unique_ptr<MeshColoringProperty> coloring_;
  std::unique_ptr<rviz_common::properties::IntProperty> max_vertices_;
  size_t coloring_revision_ = 0;
  bool change_pending_ = false;
};

}  // namespace kimera_pgmo
