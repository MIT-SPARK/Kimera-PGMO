#include "kimera_pgmo_rviz/mesh_coloring_property.h"

#include <QSignalBlocker>
#include <stdexcept>

#include <rviz_common/config.hpp>

namespace kimera_pgmo {

using rviz_common::properties::Property;

MeshColoringProperty::MeshColoringProperty(Property* parent, const QString& name)
    : EnumProperty(name, "kimera_pgmo/RGB", "Mesh coloring mode", parent),
      loader_(std::make_shared<pluginlib::ClassLoader<MeshColoringPlugin>>(
          "kimera_pgmo_rviz", "kimera_pgmo::MeshColoringPlugin")) {
  for (const auto& type : loader_->getDeclaredClasses()) {
    addOption(QString::fromStdString(type));
  }

  connect(this, &Property::changed, this, &MeshColoringProperty::selectColoring);
  selectColoring();
}

MeshColoringProperty::~MeshColoringProperty() = default;

void MeshColoringProperty::ensureEditor(const std::string& type) {
  if (editors_.count(type)) {
    return;
  }

  Editor editor;
  editor.plugin = loader_->createSharedInstance(type);
  editor.properties = std::make_unique<Property>(QString::fromStdString(type),
                                                 QVariant(),
                                                 "Settings for this coloring mode",
                                                 this);
  editor.plugin->createProperties(editor.properties.get());
  connect(editor.plugin.get(), &MeshColoringPlugin::changed, this, [this, type]() {
    if (getStdString() == type) {
      Q_EMIT coloringChanged();
    }
  });
  editors_.emplace(type, std::move(editor));
}

void MeshColoringProperty::selectColoring() {
  error_.clear();
  const auto type = getStdString();
  try {
    ensureEditor(type);
  } catch (const std::exception& e) {
    error_ = e.what();
  }

  for (const auto& [name, editor] : editors_) {
    editor.properties->setHidden(name != type);
  }

  Q_EMIT coloringChanged();
}

void MeshColoringProperty::load(const rviz_common::Config& config) {
  {
    const QSignalBlocker blocker(this);
    // Property::load only visits existing children. Create all saved editors first,
    // including inactive modes and recursively saved children of Split coloring.
    for (const auto& type : loader_->getDeclaredClasses()) {
      if (!config.mapGetChild(QString::fromStdString(type)).isValid()) {
        continue;
      }

      try {
        ensureEditor(type);
      } catch (const std::exception& e) {
        error_ = e.what();
      }
    }

    EnumProperty::load(config);
  }

  selectColoring();
}

std::shared_ptr<MeshColoring> MeshColoringProperty::createColoring() const {
  if (!error_.empty()) {
    throw std::runtime_error(error_);
  }

  const auto iter = editors_.find(getStdString());
  if (iter == editors_.end()) {
    throw std::runtime_error("No coloring plugin selected");
  }

  auto processor = iter->second.plugin->createColoring();
  if (!processor) {
    throw std::runtime_error("Coloring plugin returned an empty processor");
  }

  // A processor may outlive its editor's property tree. Retain the plugin instance
  // and loader until its destructor has run, including for downstream libraries.
  return {processor.release(),
          [loader = loader_, plugin = iter->second.plugin](MeshColoring* processor) {
            delete processor;
          }};
}

}  // namespace kimera_pgmo
