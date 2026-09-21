#pragma once
#include <map>
#include <memory>
#include <string>

#include <pluginlib/class_loader.hpp>
#include <rviz_common/properties/enum_property.hpp>

#include "kimera_pgmo_rviz/mesh_coloring_plugin.h"

namespace kimera_pgmo {

//! Selects a coloring editor and retains the properties of previously used modes.
//! May also be embedded in a downstream editor to compose coloring processors.
class MeshColoringProperty : public rviz_common::properties::EnumProperty {
  Q_OBJECT
 public:
  explicit MeshColoringProperty(rviz_common::properties::Property* parent,
                                const QString& name = "Coloring");
  ~MeshColoringProperty() override;

  void load(const rviz_common::Config& config) override;
  std::shared_ptr<MeshColoring> createColoring() const;

 Q_SIGNALS:
  void coloringChanged();

 private:
  struct Editor {
    std::shared_ptr<MeshColoringPlugin> plugin;
    std::unique_ptr<rviz_common::properties::Property> properties;
  };

  void selectColoring();
  void ensureEditor(const std::string& type);

  std::shared_ptr<pluginlib::ClassLoader<MeshColoringPlugin>> loader_;
  std::map<std::string, Editor> editors_;
  std::string error_;
};

}  // namespace kimera_pgmo
