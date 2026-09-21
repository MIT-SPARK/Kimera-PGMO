/**
 * @file   mesh_visual.cpp
 * @brief  Rviz visuals for rendering mesh
 * @author Nathan Hughes
 */
#include "kimera_pgmo_rviz/mesh_visual.h"

#include <OgreManualObject.h>
#include <OgreMaterialManager.h>
#include <OgreSceneManager.h>
#include <OgreSceneNode.h>
#include <kimera_pgmo_ros/conversion/mesh_types.h>
#include <kimera_pgmo_ros/mesh_coloring_factories.h>

#include <Eigen/Dense>
#include <rviz_common/logging.hpp>

namespace kimera_pgmo {

std::atomic<uint32_t> MeshVisual::visual_id_ = 0;

MeshVisual::MeshVisual(Ogre::SceneManager* manager,
                       Ogre::SceneNode* parent,
                       const std::string& ns)
    : visual_ns_(ns),
      ambient_(0.9, 0.9, 0.9, 0.9),
      emissive_(0.1, 0.1, 0.1, 0.1),
      diffuse_(0.05, 0.05, 0.05, 0.05),
      specular_(0.0, 0.0, 0.0, 0.0),
      manager_(manager),
      node_(nullptr) {
  node_ = parent->createChildSceneNode();
  node_->setVisible(false);
  const uint32_t my_id = visual_id_.fetch_add(1);
  mesh_name_ = "kimera_pgmo_mesh_visual_" + std::to_string(my_id);
  material_name_ = "kimera_pgmo_mesh_material_" + std::to_string(my_id);

  auto& material_manager = Ogre::MaterialManager::getSingleton();
  auto material = material_manager.create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setReceiveShadows(false);

  setCullMode();
  setLightingMode();
  coloring_ = makeRgbColoring();
}

MeshVisual::~MeshVisual() {
  reset();

  auto& material_manager = Ogre::MaterialManager::getSingleton();
  material_manager.unload(material_name_);
  material_manager.remove(material_name_);

  manager_->destroySceneNode(node_);
}

void MeshVisual::reset() {
  for (const auto mesh : meshes_) {
    manager_->destroyManualObject(mesh);
  }

  meshes_.clear();
  geometry_.setMesh({}, {});
  colors_.clear();
  transparent_vertices_ = 0;
  updateTransparency();
}

void MeshVisual::setPose(const Ogre::Vector3& parent_t_mesh,
                         const Ogre::Quaternion& parent_R_mesh) {
  RVIZ_COMMON_LOG_DEBUG_STREAM("Setting pose for mesh '"
                               << visual_ns_ << "' to pos: " << parent_t_mesh
                               << " rot: " << parent_R_mesh);
  node_->setPosition(parent_t_mesh);
  node_->setOrientation(parent_R_mesh);
}

void MeshVisual::setColoring(std::shared_ptr<MeshColoring> coloring, size_t revision) {
  const MeshColoringView view{geometry_.vertices(), geometry_.faces(), visual_ns_};
  coloring->prepare(view, 0);
  std::vector<traits::Color> colors;
  colors.reserve(view.vertices.size());
  size_t transparent_vertices = 0;
  for (size_t i = 0; i < view.vertices.size(); ++i) {
    colors.push_back(coloring->color(view, i));
    if (colors.back()[3] < 255) {
      ++transparent_vertices;
    }
  }

  coloring_ = std::move(coloring);
  colors_ = std::move(colors);
  transparent_vertices_ = transparent_vertices;
  updateTransparency();
  coloring_revision_ = revision;
  geometry_.markAllDirty();
  upload();
}

void MeshVisual::setMaxVertices(size_t max_vertices) {
  geometry_.setMaxVertices(max_vertices);
  upload();
}

void MeshVisual::setMessage(const kimera_pgmo_msgs::msg::Mesh& mesh) {
  std::vector<traits::Vertex> vertices;
  vertices.reserve(mesh.vertices.size());
  for (const auto& vertex : mesh.vertices) {
    vertices.push_back(conversions::from_ros(vertex));
  }

  std::vector<traits::Face> faces;
  faces.reserve(mesh.triangles.size());
  for (const auto& face : mesh.triangles) {
    faces.push_back(conversions::from_ros(face));
  }

  geometry_.setMesh(std::move(vertices), std::move(faces));
  updateColors(0);
  upload();
}

void MeshVisual::applyDelta(const MeshDelta& delta) {
  const auto first_changed = geometry_.applyDelta(delta);
  updateColors(first_changed);
  upload();
}

void MeshVisual::updateColors(size_t first_changed) {
  const MeshColoringView view{geometry_.vertices(), geometry_.faces(), visual_ns_};
  if (coloring_->prepare(view, first_changed)) {
    first_changed = 0;
    geometry_.markAllDirty();
  }

  for (size_t i = view.vertices.size(); i < colors_.size(); ++i) {
    if (colors_[i][3] < 255) {
      --transparent_vertices_;
    }
  }

  colors_.resize(view.vertices.size(), {0, 0, 0, 255});
  for (size_t i = first_changed; i < colors_.size(); ++i) {
    const auto color = coloring_->color(view, i);
    if (colors_[i][3] < 255) {
      --transparent_vertices_;
    }

    colors_[i] = color;
    if (colors_[i][3] < 255) {
      ++transparent_vertices_;
    }
  }

  updateTransparency();
}

void MeshVisual::updateTransparency() {
  const auto enabled = transparent_vertices_ != 0;
  if (transparency_enabled_ == enabled) {
    return;
  }

  transparency_enabled_ = enabled;
  auto& material = getMaterial();
  material.setSceneBlending(enabled ? Ogre::SBT_TRANSPARENT_ALPHA : Ogre::SBT_REPLACE);
  material.setDepthWriteEnabled(!enabled);
}

void MeshVisual::upload() {
  const auto& chunks = geometry_.chunks();
  while (meshes_.size() > chunks.size()) {
    manager_->destroyManualObject(meshes_.back());
    meshes_.pop_back();
  }

  while (meshes_.size() < chunks.size()) {
    auto mesh =
        manager_->createManualObject(mesh_name_ + "_" + std::to_string(meshes_.size()));
    mesh->setDynamic(true);
    mesh->setVisible(visible_);
    node_->attachObject(mesh);
    meshes_.push_back(mesh);
  }

  for (size_t i = 0; i < chunks.size(); ++i) {
    if (chunks[i].dirty) {
      uploadChunk(i);
    }
  }

  geometry_.clearDirty();
}

void MeshVisual::uploadChunk(size_t index) {
  const auto& chunk = geometry_.chunks()[index];
  auto mesh = meshes_[index];
  mesh->estimateVertexCount(chunk.vertices.size());
  mesh->estimateIndexCount(3 * chunk.faces.size());
  if (!mesh->getSections().empty()) {
    mesh->beginUpdate(0);
  } else {
    mesh->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);
  }

  Ogre::AxisAlignedBox bounds;
  for (const auto vertex : chunk.vertices) {
    const auto& pos = geometry_.vertices()[vertex].pos;
    const auto normal = geometry_.normal(vertex);
    const auto& color = colors_[vertex];
    bounds.merge(Ogre::Vector3(pos.x(), pos.y(), pos.z()));
    mesh->position(pos.x(), pos.y(), pos.z());
    mesh->normal(normal.x(), normal.y(), normal.z());
    mesh->colour(
        color[0] / 255.0f, color[1] / 255.0f, color[2] / 255.0f, color[3] / 255.0f);
  }

  for (const auto& face : chunk.faces) {
    mesh->triangle(face[0], face[1], face[2]);
  }

  mesh->end();
  mesh->setBoundingBox(bounds);
}

void MeshVisual::shouldCull(bool cull) {
  cull_faces_ = cull;
  setCullMode();
}

void MeshVisual::shouldLight(bool light) {
  lighting_enabled_ = light;
  setLightingMode();
  geometry_.setNormalsEnabled(light);
  upload();
}

void MeshVisual::setVisible(bool visible) {
  visible_ = visible;
  node_->setVisible(visible_);
}

Ogre::Material& MeshVisual::getMaterial() const {
  auto& material_manager = Ogre::MaterialManager::getSingleton();
  auto material = material_manager.getByName(material_name_);
  return *material;
}

void MeshVisual::setCullMode() {
  auto& material = getMaterial();
  RVIZ_COMMON_LOG_DEBUG_STREAM("Setting culling for " << material_name_ << ": "
                                                      << std::boolalpha << cull_faces_);
  material.setCullingMode(cull_faces_ ? Ogre::CULL_CLOCKWISE : Ogre::CULL_NONE);
}

void MeshVisual::setLightingMode() {
  auto& material = getMaterial();
  RVIZ_COMMON_LOG_DEBUG_STREAM("Setting lighting for " << material_name_ << ": "
                                                       << std::boolalpha
                                                       << lighting_enabled_);

  material.setAmbient(ambient_);
  material.setDiffuse(diffuse_);
  material.setSpecular(specular_);
  material.setShininess(2.0);  // this turns shininess down (max 128)
  if (lighting_enabled_) {
    material.getTechnique(0)->setLightingEnabled(true);
    const auto pass = material.getTechnique(0)->getPass(0);
    pass->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_EMISSIVE |
                                  Ogre::TVC_DIFFUSE | Ogre::TVC_SPECULAR);
    pass->setEmissive(emissive_);
  } else {
    material.getTechnique(0)->setLightingEnabled(false);
    const auto pass = material.getTechnique(0)->getPass(0);
    pass->setVertexColourTracking(Ogre::TVC_NONE);
  }
}

void MeshVisual::setLighting(Ogre::ColourValue ambient,
                             Ogre::ColourValue emissive,
                             Ogre::ColourValue diffuse,
                             Ogre::ColourValue specular) {
  ambient_ = ambient;
  emissive_ = emissive;
  diffuse_ = diffuse;
  specular_ = specular;
  setLightingMode();
}

}  // namespace kimera_pgmo
