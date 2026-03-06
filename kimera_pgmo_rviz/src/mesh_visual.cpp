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

#include <Eigen/Dense>
#include <rviz_common/logging.hpp>

namespace kimera_pgmo {

using Matrix4Xf = Eigen::Matrix<float, 4, Eigen::Dynamic>;
using kimera_pgmo_msgs::msg::Mesh;

std::atomic<uint32_t> MeshVisual::visual_id_ = 0;

namespace {

inline void fillVec(const geometry_msgs::msg::Point& p, Eigen::Vector3f& v) {
  v << p.x, p.y, p.z;
}

inline void updateNormal(const Eigen::Vector3f& n,
                         size_t index,
                         Eigen::Matrix4Xf& normals) {
  normals.block<3, 1>(0, index) += n;
  normals(3, index) += 1;
}

}  // namespace

MeshVisual::MeshVisual(Ogre::SceneManager* manager,
                       Ogre::SceneNode* parent,
                       const std::string& ns)
    : visual_ns_(ns),
      ambient_(0.9, 0.9, 0.9, 0.9),
      emissive_(0.1, 0.1, 0.1, 0.1),
      diffuse_(0.05, 0.05, 0.05, 0.05),
      specular_(0.0, 0.0, 0.0, 0.0),
      manager_(manager),
      node_(nullptr),
      mesh_(nullptr) {
  node_ = parent->createChildSceneNode();
  node_->setVisible(false);
  const uint32_t my_id = visual_id_.fetch_add(1);
  mesh_name_ = "kimera_pgmo_mesh_visual_" + std::to_string(my_id);
  material_name_ = "kimera_pgmo_mesh_material_" + std::to_string(my_id);

  auto& material_manager = Ogre::MaterialManager::getSingleton();
  auto material = material_manager.create(
      material_name_, Ogre::ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
  material->setReceiveShadows(false);
}

MeshVisual::~MeshVisual() {
  if (mesh_) {
    manager_->destroyManualObject(mesh_);
  }

  auto& material_manager = Ogre::MaterialManager::getSingleton();
  material_manager.unload(material_name_);
  material_manager.remove(material_name_);

  manager_->destroySceneNode(node_);
}

void MeshVisual::reset() {
  if (mesh_) {
    mesh_->clear();
  }
}

void MeshVisual::setPose(const Ogre::Vector3& parent_t_mesh,
                         const Ogre::Quaternion& parent_R_mesh) {
  RVIZ_COMMON_LOG_DEBUG_STREAM("Setting pose for mesh '"
                               << visual_ns_ << "' to pos: " << parent_t_mesh
                               << " rot: " << parent_R_mesh);
  node_->setPosition(parent_t_mesh);
  node_->setOrientation(parent_R_mesh);
}

void MeshVisual::setMesh(const std::vector<traits::Vertex>& vertices,
                         const std::vector<traits::Face>& faces) {
  RVIZ_COMMON_LOG_DEBUG_STREAM("Setting mesh with " << vertices.size()
                                                    << " vertices and " << faces.size()
                                                    << " faces");
  RVIZ_COMMON_LOG_DEBUG_STREAM("Names: mesh=" << mesh_name_
                                              << ", material=" << material_name_);

  reset();
  if (!mesh_) {
    mesh_ = manager_->createManualObject(mesh_name_);
    setCullMode();
    setLightingMode();
    node_->attachObject(mesh_);
  }

  Eigen::Matrix4Xf normals = Eigen::Matrix4Xf::Zero(4, vertices.size());
  mesh_->estimateVertexCount(vertices.size());
  mesh_->estimateIndexCount(3 * faces.size());
  mesh_->setDynamic(false);
  mesh_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for (const auto& face : faces) {
    if (face[0] >= vertices.size() || face[1] >= vertices.size() ||
        face[2] >= vertices.size()) {
      continue;
    }

    mesh_->triangle(face[0], face[1], face[2]);

    // TODO(nathan) do this incrementally and pass in
    const auto& p1 = vertices[face[0]].pos;
    const auto& p2 = vertices[face[1]].pos;
    const auto& p3 = vertices[face[2]].pos;
    Eigen::Vector3f n = ((p2 - p1).cross(p3 - p1)).normalized();
    updateNormal(n, face[0], normals);
    updateNormal(n, face[1], normals);
    updateNormal(n, face[2], normals);
  }

  for (size_t i = 0; i < vertices.size(); ++i) {
    const auto& p = vertices[i];
    mesh_->position(p.pos.x(), p.pos.y(), p.pos.z());
    const Eigen::Vector4f n = normals.block<4, 1>(0, i);
    if (n[3] == 0.0f) {
      // not touched by any faces so default normal doesn't matter
      mesh_->normal(0.0, 0.0, 1.0);
    } else {
      mesh_->normal(n.x() / n[3], n.y() / n[3], n.z() / n[3]);
    }

    if (p.traits.properties.has_color) {
      mesh_->colour(p.traits.color[0] / 255.0f,
                    p.traits.color[1] / 255.0f,
                    p.traits.color[2] / 255.0f,
                    p.traits.color[3] / 255.0f);
    }
  }

  mesh_->end();
}

void MeshVisual::setMessage(const Mesh& mesh) {
  RVIZ_COMMON_LOG_DEBUG_STREAM("Setting mesh with "
                               << mesh.vertices.size() << " vertices and "
                               << mesh.triangles.size() << " faces");
  RVIZ_COMMON_LOG_DEBUG_STREAM("Names: mesh=" << mesh_name_
                                              << ", material=" << material_name_);

  reset();

  if (!mesh_) {
    mesh_ = manager_->createManualObject(mesh_name_);
    setCullMode();
    setLightingMode();
    node_->attachObject(mesh_);
  }

  Eigen::Matrix4Xf normals = Eigen::Matrix4Xf::Zero(4, mesh.vertices.size());

  mesh_->estimateVertexCount(mesh.vertices.size());
  mesh_->estimateIndexCount(3 * mesh.triangles.size());
  mesh_->setDynamic(false);
  mesh_->begin(material_name_, Ogre::RenderOperation::OT_TRIANGLE_LIST);

  for (const auto& face : mesh.triangles) {
    const auto& triangle = face.vertex_indices;
    if (triangle[0] >= mesh.vertices.size() || triangle[1] >= mesh.vertices.size() ||
        triangle[2] >= mesh.vertices.size()) {
      continue;
    }

    mesh_->triangle(triangle[0], triangle[1], triangle[2]);
    Eigen::Vector3f p1;
    fillVec(mesh.vertices[triangle[0]].pos, p1);
    Eigen::Vector3f p2;
    fillVec(mesh.vertices[triangle[1]].pos, p2);
    Eigen::Vector3f p3;
    fillVec(mesh.vertices[triangle[2]].pos, p3);
    Eigen::Vector3f n = ((p2 - p1).cross(p3 - p1)).normalized();
    updateNormal(n, triangle[0], normals);
    updateNormal(n, triangle[1], normals);
    updateNormal(n, triangle[2], normals);
  }

  for (size_t i = 0; i < mesh.vertices.size(); ++i) {
    const auto& p = mesh.vertices[i];
    mesh_->position(p.pos.x, p.pos.y, p.pos.z);
    const Eigen::Vector4f n = normals.block<4, 1>(0, i);
    if (n[3] == 0.0f) {
      // not touched by any faces so default normal doesn't matter
      mesh_->normal(0.0, 0.0, 1.0);
    } else {
      mesh_->normal(n.x() / n[3], n.y() / n[3], n.z() / n[3]);
    }

    // TODO(nathan) this is technically guaranteed to be true for all vertices, but...
    if (p.has_color) {
      mesh_->colour(p.color.r, p.color.g, p.color.b, p.color.a);
    }
  }

  mesh_->end();
}

void MeshVisual::shouldCull(bool cull) {
  cull_faces_ = cull;
  setCullMode();
}

void MeshVisual::shouldLight(bool light) {
  lighting_enabled_ = light;
  setLightingMode();
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
