/**
 * @file   mesh_visual.h
 * @brief  Rviz visual for viewing mesh
 * @author Nathan Hughes
 */
#pragma once
#include <Ogre.h>
#include <kimera_pgmo/mesh_types.h>

#include <atomic>
#include <string>

#include <kimera_pgmo_msgs/msg/mesh.hpp>

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

  void setMesh(const std::vector<traits::Vertex>& vertices,
               const std::vector<traits::Face>& faces);

  void setMessage(const kimera_pgmo_msgs::msg::Mesh& mesh);

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
  Ogre::ManualObject* mesh_;
};

}  // namespace kimera_pgmo
