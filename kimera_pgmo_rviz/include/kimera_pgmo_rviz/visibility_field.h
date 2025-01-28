#pragma once

#include <rviz/properties/bool_property.h>

namespace kimera_pgmo {

class MeshDisplay;

/**
 * @brief Property for managing visibility of individual meshes in the mesh display. The
 * visibility fields are managed hierarchically by namespace.
 */
class VisibilityField : public rviz::BoolProperty {
  Q_OBJECT
 public:
  VisibilityField(const std::string& name,
                  rviz::BoolProperty* parent,
                  MeshDisplay* master);
  ~VisibilityField() override;

  /**
   * @brief Insert a field into the visibility tree.
   * @param field_name The full name of the field to insert.
   */
  void addField(const std::string& field_name);
  /**
   * @brief Remove a field from the visibility tree.
   * @param field_name The full name of the field to remove.
   */
  void removeField(const std::string& field_name);

  /**
   * @brief Check if a field is enabled.
   * @param field_name The full name of the field to check.
   */
  bool isEnabled(const std::string& field_name);

  /**
   * @brief Set the enabled state of all fields in the hierarchy below and including
   * this field.
   * @param enabled The state to set.
   */
  void setEnabledForAll(bool enabled);

 private:
  Q_SLOT void visibleSlot();

  // The master mesh display.
  MeshDisplay* master_;

  // <namespace - next fields in the hierarchy>
  std::unordered_map<std::string, std::unique_ptr<VisibilityField>> children_;

  /**
   * @brief Check if the name contains a namespace and split it into the namespace and
   * the subname.
   * @param name The name to check.
   * @param ns The first identified namespace.
   * @param sub_name The remainder of the name excluding the namespace. This may include
   * further namespaces.
   */
  bool hasNameSpace(const std::string& name, std::string& ns, std::string& sub_name);
};

}  // namespace kimera_pgmo