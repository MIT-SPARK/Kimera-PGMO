#include <kimera_pgmo_ros/mesh_coloring_factories.h>
#include <yaml-cpp/yaml.h>

#include <cmath>
#include <stdexcept>

#include <pluginlib/class_list_macros.hpp>
#include <rviz_common/properties/color_property.hpp>
#include <rviz_common/properties/enum_property.hpp>
#include <rviz_common/properties/file_picker_property.hpp>
#include <rviz_common/properties/float_property.hpp>
#include <rviz_common/properties/string_property.hpp>
#include <rviz_common/properties/vector_property.hpp>

#include "kimera_pgmo_rviz/mesh_coloring_plugin.h"
#include "kimera_pgmo_rviz/mesh_coloring_property.h"

namespace kimera_pgmo {
namespace {

using rviz_common::properties::ColorProperty;
using rviz_common::properties::EnumProperty;
using rviz_common::properties::FilePickerProperty;
using rviz_common::properties::FloatProperty;
using rviz_common::properties::Property;
using rviz_common::properties::StringProperty;
using rviz_common::properties::VectorProperty;

traits::Color colorValue(const ColorProperty& property, double opacity = 1.0) {
  if (!std::isfinite(opacity) || opacity < 0.0 || opacity > 1.0) {
    throw std::invalid_argument("Opacity must be in [0, 1]");
  }

  const auto color = property.getColor();
  return {static_cast<uint8_t>(color.red()),
          static_cast<uint8_t>(color.green()),
          static_cast<uint8_t>(color.blue()),
          static_cast<uint8_t>(std::lround(255.0 * opacity))};
}

ColorProperty* addColor(Property* parent,
                        MeshColoringPlugin* owner,
                        const QString& name,
                        const QColor& initial = QColor(102, 102, 102)) {
  auto property =
      new ColorProperty(name, initial, "Color used by this coloring mode", parent);
  QObject::connect(property, &Property::changed, owner, &MeshColoringPlugin::changed);
  return property;
}

FloatProperty* addRatio(Property* parent,
                        MeshColoringPlugin* owner,
                        const QString& name,
                        const QString& description) {
  auto property = new FloatProperty(name, 1.0, description, parent);
  property->setMin(0.0);
  property->setMax(1.0);
  QObject::connect(property, &Property::changed, owner, &MeshColoringPlugin::changed);
  return property;
}

MeshPalette readPalette(const std::string& path) {
  const auto node = YAML::LoadFile(path);
  if (!node.IsMap()) {
    throw std::invalid_argument(
        "Palette file must map integer labels to RGB/RGBA byte arrays");
  }

  MeshPalette palette;
  for (const auto& entry : node) {
    const auto& channels = entry.second;
    if (!channels.IsSequence() || (channels.size() != 3 && channels.size() != 4)) {
      throw std::invalid_argument("Palette colors must contain three or four channels");
    }

    traits::Color color{0, 0, 0, 255};
    for (size_t i = 0; i < channels.size(); ++i) {
      const auto value = channels[i].as<int>();
      if (value < 0 || value > 255) {
        throw std::invalid_argument("Palette color channels must be in [0, 255]");
      }

      color[i] = static_cast<uint8_t>(value);
    }

    palette[entry.first.as<traits::Label>()] = color;
  }

  return palette;
}

traits::Timestamp timestampValue(const StringProperty& property) {
  const auto text = property.getString();
  bool valid = false;
  const auto value = text.toULongLong(&valid);
  if (!valid || text.isEmpty() || text.startsWith('-')) {
    throw std::invalid_argument(property.getName().toStdString() +
                                " must be an unsigned integer in nanoseconds");
  }

  return value;
}

//! Shared editor for timestamp ranges; each concrete plugin owns its own instance.
struct TimeProperties {
  void create(Property* parent, MeshColoringPlugin* owner) {
    range = new EnumProperty("Range",
                             "Automatic",
                             "Automatic bounds are computed independently for each "
                             "mesh; fixed bounds avoid global recoloring",
                             parent);
    range->addOption("Automatic");
    range->addOption("Fixed");
    // StringProperty preserves all 64 timestamp bits; RViz's IntProperty is 32-bit
    // and FloatProperty would lose precision for absolute nanosecond timestamps.
    minimum = new StringProperty(
        "Minimum Time (ns)", "0", "Exact unsigned nanoseconds", parent);
    maximum = new StringProperty(
        "Maximum Time (ns)", "1000000000", "Exact unsigned nanoseconds", parent);
    invalid = addColor(parent, owner, "Invalid Color", QColor(0, 255, 0));
    QObject::connect(range, &Property::changed, owner, [this, owner]() {
      updateVisibility();
      Q_EMIT owner->changed();
    });
    QObject::connect(minimum, &Property::changed, owner, &MeshColoringPlugin::changed);
    QObject::connect(maximum, &Property::changed, owner, &MeshColoringPlugin::changed);
    updateVisibility();
  }

  void updateVisibility() {
    const auto automatic = range->getString() == "Automatic";
    minimum->setHidden(automatic);
    maximum->setHidden(automatic);
  }

  std::optional<MeshTimeBounds> bounds() const {
    if (range->getString() == "Automatic") {
      return std::nullopt;
    }

    if (range->getString() != "Fixed") {
      throw std::invalid_argument("Range must be Automatic or Fixed");
    }

    return MeshTimeBounds{timestampValue(*minimum), timestampValue(*maximum)};
  }

  EnumProperty* range = nullptr;
  StringProperty* minimum = nullptr;
  StringProperty* maximum = nullptr;
  ColorProperty* invalid = nullptr;
};

}  // namespace

class RgbColoringPlugin : public MeshColoringPlugin {
 public:
  void createProperties(Property* parent) override {
    fallback_ = addColor(parent, this, "Default Color");
  }

  std::unique_ptr<MeshColoring> createColoring() const override {
    return makeRgbColoring({colorValue(*fallback_)});
  }

 private:
  ColorProperty* fallback_ = nullptr;
};

class UniformColoringPlugin : public MeshColoringPlugin {
 public:
  void createProperties(Property* parent) override {
    color_ = addColor(parent, this, "Color");
    opacity_ = addRatio(parent, this, "Opacity", "Vertex alpha in [0, 1]");
  }

  std::unique_ptr<MeshColoring> createColoring() const override {
    return makeUniformColoring({colorValue(*color_, opacity_->getFloat())});
  }

 private:
  ColorProperty* color_ = nullptr;
  FloatProperty* opacity_ = nullptr;
};

class SemanticColoringPlugin : public MeshColoringPlugin {
 public:
  void createProperties(Property* parent) override {
    palette_ =
        new EnumProperty("Palette", "Distinct 150", "Semantic label palette", parent);
    palette_->addOption("Distinct 150");
    palette_->addOption("File");
    file_ = new FilePickerProperty(
        "Palette File",
        "",
        "YAML map from integer labels to RGB/RGBA bytes, e.g. {5: [255, 0, 0]}",
        parent);
    file_->setHidden(true);
    alpha_ = addRatio(parent,
                      this,
                      "Label Blend",
                      "Blend between source RGB (0) and semantic colors (1)");
    fallback_ = addColor(parent, this, "Default Color");
    connect(palette_, &Property::changed, this, [this]() {
      file_->setHidden(palette_->getString() != "File");
      Q_EMIT changed();
    });
    connect(file_, &Property::changed, this, &MeshColoringPlugin::changed);
  }

  std::unique_ptr<MeshColoring> createColoring() const override {
    SemanticColoringConfig config;
    config.default_color = colorValue(*fallback_);
    config.alpha = alpha_->getFloat();
    if (palette_->getString() == "File") {
      config.palette = readPalette(file_->getStdString());
    } else if (palette_->getString() != "Distinct 150") {
      throw std::invalid_argument("Unknown semantic palette");
    }

    return makeSemanticColoring(config);
  }

 private:
  EnumProperty* palette_ = nullptr;
  FilePickerProperty* file_ = nullptr;
  FloatProperty* alpha_ = nullptr;
  ColorProperty* fallback_ = nullptr;
};

class FirstSeenColoringPlugin : public MeshColoringPlugin {
 public:
  void createProperties(Property* parent) override { properties_.create(parent, this); }

  std::unique_ptr<MeshColoring> createColoring() const override {
    return makeFirstSeenColoring(
        {colorValue(*properties_.invalid), properties_.bounds()});
  }

 private:
  TimeProperties properties_;
};

class LastSeenColoringPlugin : public MeshColoringPlugin {
 public:
  void createProperties(Property* parent) override { properties_.create(parent, this); }

  std::unique_ptr<MeshColoring> createColoring() const override {
    return makeLastSeenColoring(
        {colorValue(*properties_.invalid), properties_.bounds()});
  }

 private:
  TimeProperties properties_;
};

class SeenDurationColoringPlugin : public MeshColoringPlugin {
 public:
  void createProperties(Property* parent) override { properties_.create(parent, this); }

  std::unique_ptr<MeshColoring> createColoring() const override {
    return makeSeenDurationColoring(
        {colorValue(*properties_.invalid), properties_.bounds()});
  }

 private:
  TimeProperties properties_;
};

class SplitColoringPlugin : public MeshColoringPlugin {
 public:
  void createProperties(Property* parent) override {
    normal_ =
        new VectorProperty("Plane Normal",
                           Ogre::Vector3::UNIT_SCALE,
                           "Nonzero normal; the positive side uses the child coloring",
                           parent);
    origin_ = new VectorProperty("Plane Origin",
                                 Ogre::Vector3::ZERO,
                                 "Point on the split plane in mesh coordinates",
                                 parent);
    fallback_ = addColor(parent, this, "Default Color");
    child_ = new MeshColoringProperty(parent, "Child Coloring");
    connect(normal_, &Property::changed, this, &MeshColoringPlugin::changed);
    connect(origin_, &Property::changed, this, &MeshColoringPlugin::changed);
    connect(child_,
            &MeshColoringProperty::coloringChanged,
            this,
            &MeshColoringPlugin::changed);
  }

  std::unique_ptr<MeshColoring> createColoring() const override {
    const auto normal = normal_->getVector();
    const auto origin = origin_->getVector();
    SplitColoringConfig config;
    config.normal = {normal.x, normal.y, normal.z};
    config.origin = {origin.x, origin.y, origin.z};
    config.default_color = colorValue(*fallback_);
    return makeSplitColoring(config, child_->createColoring());
  }

 private:
  VectorProperty* normal_ = nullptr;
  VectorProperty* origin_ = nullptr;
  ColorProperty* fallback_ = nullptr;
  MeshColoringProperty* child_ = nullptr;
};

}  // namespace kimera_pgmo

PLUGINLIB_EXPORT_CLASS(kimera_pgmo::RgbColoringPlugin, kimera_pgmo::MeshColoringPlugin)
PLUGINLIB_EXPORT_CLASS(kimera_pgmo::UniformColoringPlugin,
                       kimera_pgmo::MeshColoringPlugin)
PLUGINLIB_EXPORT_CLASS(kimera_pgmo::SemanticColoringPlugin,
                       kimera_pgmo::MeshColoringPlugin)
PLUGINLIB_EXPORT_CLASS(kimera_pgmo::FirstSeenColoringPlugin,
                       kimera_pgmo::MeshColoringPlugin)
PLUGINLIB_EXPORT_CLASS(kimera_pgmo::LastSeenColoringPlugin,
                       kimera_pgmo::MeshColoringPlugin)
PLUGINLIB_EXPORT_CLASS(kimera_pgmo::SeenDurationColoringPlugin,
                       kimera_pgmo::MeshColoringPlugin)
PLUGINLIB_EXPORT_CLASS(kimera_pgmo::SplitColoringPlugin,
                       kimera_pgmo::MeshColoringPlugin)
