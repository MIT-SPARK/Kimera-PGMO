#include "kimera_pgmo_rviz/visuals_map.h"

#include <memory>
#include <unordered_map>

#include "kimera_pgmo_rviz/mesh_visual.h"

namespace kimera_pgmo {

MeshVisual* VisualsMap::get(const std::string& ns) {
  const auto it = visuals_.find(ns);
  if (it == visuals_.end()) {
    return nullptr;
  }
  return it->second.get();
}

MeshVisual& VisualsMap::create(const std::string& ns,
                               Ogre::SceneManager* manager,
                               Ogre::SceneNode* parent) {
  return *visuals_
              .insert(
                  std::make_pair(ns, std::make_unique<MeshVisual>(manager, parent, ns)))
              .first->second;
}

void VisualsMap::deleteVisual(const std::string& ns) { visuals_.erase(ns); }

void VisualsMap::reset() { visuals_.clear(); }

void VisualsMap::applyToAll(std::function<void(MeshVisual&)> func) {
  for (auto& visual : visuals_) {
    func(*visual.second);
  }
}

}  // namespace kimera_pgmo