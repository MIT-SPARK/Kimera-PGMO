/**
 * @file   mesh_visual.h
 * @brief  Rviz visual for viewing mesh
 * @author Nathan Hughes
 */
#pragma once
#include <Ogre.h>
#include <kimera_pgmo_ros/mesh_coloring.h>

#include <atomic>
#include <optional>
#include <string>

#include <kimera_pgmo_msgs/msg/mesh.hpp>

#include "kimera_pgmo_rviz/mesh_geometry.h"

namespace Ogre {
class SceneManager;
class SceneNode;
class ManualObject;
class Quaternion;
class Material;
}  // namespace Ogre

namespace kimera_pgmo {

class MeshVisual {
 public:
  MeshVisual(Ogre::SceneManager* manager,
             Ogre::SceneNode* parent,
             const std::string& ns);

  virtual ~MeshVisual();

  void setPose(const Ogre::Vector3& parent_t_mesh,
               const Ogre::Quaternion& parent_R_mesh);

  void setMessage(const kimera_pgmo_msgs::msg::Mesh& mesh);
  void applyDelta(const MeshDelta& delta);
  void setColoring(std::shared_ptr<MeshColoring> coloring, size_t revision);
  std::optional<size_t> coloringRevision() const { return coloring_revision_; }
  void setMaxVertices(size_t max_vertices);

  void shouldCull(bool cull);
  void shouldLight(bool light);
  void setVisible(bool visible);

  void setLighting(Ogre::ColourValue ambient,
                   Ogre::ColourValue emissive,
                   Ogre::ColourValue diffuse,
                   Ogre::ColourValue specular);

  bool isVisible() const { return visible_; }

  void reset();

 private:
  void updateColors(size_t first_changed);
  void updateTransparency();
  void upload();
  void uploadChunk(size_t index);
  void setCullMode();
  void setLightingMode();

  Ogre::Material& getMaterial() const;

  static std::atomic<uint32_t> visual_id_;
  std::string mesh_name_;
  std::string material_name_;
  const std::string visual_ns_;  // Namespace used to refer to this visual.

  bool cull_faces_ = false;
  bool lighting_enabled_ = false;
  bool visible_ = false;  // Whether the visual should be visible.

  Ogre::ColourValue ambient_;
  Ogre::ColourValue emissive_;
  Ogre::ColourValue diffuse_;
  Ogre::ColourValue specular_;

  Ogre::SceneManager* manager_;
  Ogre::SceneNode* node_;
  MeshGeometry geometry_;
  std::vector<Ogre::ManualObject*> meshes_;
  std::vector<traits::Color> colors_;
  size_t transparent_vertices_ = 0;
  bool transparency_enabled_ = false;
  std::shared_ptr<MeshColoring> coloring_;
  std::optional<size_t> coloring_revision_;
};

}  // namespace kimera_pgmo
