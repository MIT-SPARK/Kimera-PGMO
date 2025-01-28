#include "kimera_pgmo_rviz/visibility_field.h"

#include "kimera_pgmo_rviz/mesh_display.h"

namespace kimera_pgmo {

VisibilityField::VisibilityField(const std::string& name,
                                 rviz::BoolProperty* parent,
                                 MeshDisplay* master)
    : rviz::BoolProperty(
          name.c_str(),
          true,
          "Show or hide the mesh. If the mesh is hidden but not disabled, it "
          "will persist and is incrementally built in the background.",
          parent,
          SLOT(visibleSlot())),
      master_(master) {
  setDisableChildrenIfFalse(true);
}

VisibilityField::~VisibilityField() {}

void VisibilityField::visibleSlot() { master_->updateVisible(); }

bool VisibilityField::hasNameSpace(const std::string& name,
                                   std::string& ns,
                                   std::string& sub_name) {
  std::size_t ns_indicator = name.find(MeshDisplay::kNsSeparator);
  if (ns_indicator != std::string::npos) {
    sub_name = name.substr(ns_indicator + 1);
    ns = name.substr(0, ns_indicator);
    return true;
  }
  sub_name = name;
  return false;
}

void VisibilityField::addField(const std::string& field_name) {
  std::string ns, sub_name;
  if (hasNameSpace(field_name, ns, sub_name)) {
    // If there is at least a namespace present resolve it first.
    auto it = children_.find(ns);
    if (it == children_.end()) {
      // Add the new namespace.
      it = children_.insert(std::make_pair(ns, std::unique_ptr<VisibilityField>()))
               .first;
      it->second.reset(new VisibilityField(ns, this, master_));
    }
    it->second->addField(sub_name);
  } else {
    auto it =
        children_.insert(std::make_pair(field_name, std::unique_ptr<VisibilityField>()))
            .first;
    it->second.reset(new VisibilityField(field_name, this, master_));
  }
}

void VisibilityField::removeField(const std::string& field_name) {
  std::string ns, sub_name;
  if (hasNameSpace(field_name, ns, sub_name)) {
    // If there is at least one namespace present, resolve it first.
    const auto it = children_.find(ns);
    if (it != children_.end()) {
      it->second->removeField(sub_name);
      if (it->second->children_.empty()) {
        // If the namespace has no more children, remove it.
        children_.erase(it);
      }
    }
  } else {
    // No namespace means this is a leaf node.
    children_.erase(field_name);
  }
}

bool VisibilityField::isEnabled(const std::string& field_name) {
  if (!getBool()) {
    // This property and therefore all children are disabled.
    return false;
  }
  std::string ns, sub_name;
  if (hasNameSpace(field_name, ns, sub_name)) {
    // If there is at least a namespace present, resolve it first.
    auto it = children_.find(ns);
    if (it == children_.end()) {
      return false;
    }
    return it->second->isEnabled(sub_name);
  } else {
    // No namespace means this is a leaf node.
    auto it = children_.find(field_name);
    if (it == children_.end()) {
      return false;
    }
    return it->second->getBool();
  }
}

void VisibilityField::setEnabledForAll(bool enabled) {
  // Recursively set all properties.
  setBool(enabled);
  for (auto& child : children_) {
    child.second->setEnabledForAll(enabled);
  }
}

}  // namespace kimera_pgmo