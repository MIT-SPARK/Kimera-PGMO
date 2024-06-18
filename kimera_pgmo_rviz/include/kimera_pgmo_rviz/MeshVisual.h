/**
 * @file   MeshVisual.h
 * @brief  Rviz visual for viewing mesh
 * @author Nathan Hughes
 */
#pragma once
#include <kimera_pgmo/KimeraPgmoMesh.h>

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
  MeshVisual(Ogre::SceneManager* manager, Ogre::SceneNode* parent);
  virtual ~MeshVisual();

  void setMessage(const kimera_pgmo::KimeraPgmoMesh& mesh,
                  const Ogre::Vector3& parent_t_mesh,
                  const Ogre::Quaternion& parent_R_mesh);

  void shouldCull(bool cull);
  void shouldLight(bool light);

 private:
  void setCullMode();
  void setLightingMode();

  Ogre::Material& getMaterial() const;

  static std::atomic<uint32_t> visual_id_;
  std::string mesh_name_;
  std::string material_name_;

  bool cull_faces_;
  bool lighting_enabled_;

  Ogre::SceneManager* manager_;
  Ogre::SceneNode* node_;
  Ogre::ManualObject* mesh_;
};

}  // namespace kimera_pgmo
