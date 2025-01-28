/**
 * @file   mesh_visual.h
 * @brief  Rviz visual for viewing mesh
 * @author Nathan Hughes
 */
#pragma once
#include <kimera_pgmo_msgs/KimeraPgmoMesh.h>

#include <atomic>
#include <string>
#include <vector>

namespace Ogre {
class SceneManager;
class SceneNode;
class ManualObject;
class Vector3;
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

  void setMessage(const kimera_pgmo_msgs::KimeraPgmoMesh& mesh);

  void shouldCull(bool cull);
  void shouldLight(bool light);
  void setVisible(bool visible);

  bool isVisible() const { return visible_; }

  void reset();

 private:
  void setCullMode();
  void setLightingMode();

  Ogre::Material& getMaterial() const;

  static std::atomic<uint32_t> visual_id_;
  std::string mesh_name_;
  std::string material_name_;
  const std::string visual_ns_;  // Namespace used to refer to this visual.

  bool cull_faces_;
  bool lighting_enabled_;
  bool visible_ = false;  // Whether the visual should be visible.

  Ogre::SceneManager* manager_;
  Ogre::SceneNode* node_;
  Ogre::ManualObject* mesh_;
};

}  // namespace kimera_pgmo
