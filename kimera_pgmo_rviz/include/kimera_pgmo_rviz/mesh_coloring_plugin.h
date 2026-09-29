#pragma once
#include <kimera_pgmo_ros/mesh_coloring.h>

#include <QObject>
#include <memory>

namespace rviz_common::properties {
class Property;
}

namespace kimera_pgmo {

//! RViz editor for a coloring processor. Export subclasses with
//! PLUGINLIB_EXPORT_CLASS(Derived, kimera_pgmo::MeshColoringPlugin) and
//! pluginlib_export_plugin_description_file(kimera_pgmo_rviz plugins.xml).
//! The plugin needs a default constructor. Calls occur on the GUI thread.
class MeshColoringPlugin : public QObject {
  Q_OBJECT
 public:
  ~MeshColoringPlugin() override;

  //! Called once. The supplied parent owns the created properties. Do not delete
  //! them in the plugin destructor. Settings persist while switching modes.
  virtual void createProperties(rviz_common::properties::Property* parent) = 0;

  //! Snapshot settings into a fresh processor with independent per-mesh caches.
  //! Throw std::exception for invalid settings. Processors must not retain the
  //! editor, its properties, or other mutable UI state.
  virtual std::unique_ptr<MeshColoring> createColoring() const = 0;

 Q_SIGNALS:
  //! Emit when settings change; the display coalesces changes before recoloring.
  void changed();
};

}  // namespace kimera_pgmo
