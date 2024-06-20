#pragma once

#include <functional>
#include <memory>
#include <unordered_map>

namespace Ogre {
class SceneManager;
class SceneNode;
}  // namespace Ogre

namespace kimera_pgmo {

class MeshVisual;

/**
 * @brief Data structure managing the individual mesh visuals.
 */
class VisualsMap {
 public:
  VisualsMap() = default;
  virtual ~VisualsMap() = default;

  /**
   * @brief Get a visual for a given namespace. If the namespace does not exist, returns
   * nullptr.
   */
  MeshVisual* get(const std::string& ns);

  /**
   * @brief Create a visual for a given namespace. If the namespace already exists, the
   * existing visual is replaced.
   */
  MeshVisual& create(const std::string& ns,
                     Ogre::SceneManager* manager,
                     Ogre::SceneNode* parent);

  /**
   * @brief Delete a visual for a given namespace.
   */
  void deleteVisual(const std::string& ns);

  /**
   * @brief Delete all existing visuals.
   */
  void reset();

  /**
   * @brief Apply a function to all existing mesh visuals.
   */
  void applyToAll(std::function<void(MeshVisual&)> func);

  // Iterator access to the visuals.
  auto begin() { return visuals_.begin(); }
  auto end() { return visuals_.end(); }

 private:
  // <namespace - visual>
  std::unordered_map<std::string, std::unique_ptr<MeshVisual>> visuals_;
};

}  // namespace kimera_pgmo