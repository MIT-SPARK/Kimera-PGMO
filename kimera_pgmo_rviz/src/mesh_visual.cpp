/**
 * @file   mesh_visual.cpp
 * @brief  Rviz visuals for rendering mesh
 * @author Nathan Hughes
 */
#include "kimera_pgmo_rviz/mesh_visual.h"

#include <OGRE/OgreManualObject.h>
#include <OGRE/OgreMaterialManager.h>
#include <OGRE/OgreSceneManager.h>
#include <OGRE/OgreSceneNode.h>
#include <ros/assert.h>

#include <Eigen/Dense>
#include <stdexcept>

namespace kimera_pgmo {

using Matrix4Xf = Eigen::Matrix<float, 4, Eigen::Dynamic>;

std::atomic<uint32_t> MeshVisual::visual_id_ = 0;

namespace {

inline void fillVec(const geometry_msgs::Point& p, Eigen::Vector3f& v) {
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
      cull_faces_(false),
      lighting_enabled_(false),
      manager_(manager),
      node_(nullptr),
      mesh_(nullptr) {
  ROS_ASSERT_MSG(manager_ != nullptr, "invalid scene manager!");
  ROS_ASSERT_MSG(parent != nullptr, "invalid parent scene node!");

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
  // TODO(lschmid): WIP, check if needed.
  if (mesh_) {
    mesh_->clear();
  }
}

void MeshVisual::setPose(const Ogre::Vector3& parent_t_mesh,
                         const Ogre::Quaternion& parent_R_mesh) {
  ROS_DEBUG_STREAM("Setting pose for mesh '" << visual_ns_
                                             << "' to pos: " << parent_t_mesh
                                             << " rot: " << parent_R_mesh);
  node_->setPosition(parent_t_mesh);
  node_->setOrientation(parent_R_mesh);
}

void MeshVisual::setMessage(const kimera_pgmo_msgs::KimeraPgmoMesh& mesh) {
  ROS_DEBUG_STREAM("Setting mesh with " << mesh.vertices.size() << " vertices and "
                                        << mesh.triangles.size() << " faces");
  ROS_DEBUG_STREAM("Names: mesh=" << mesh_name_ << ", material=" << material_name_);

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
    fillVec(mesh.vertices[triangle[0]], p1);
    Eigen::Vector3f p2;
    fillVec(mesh.vertices[triangle[1]], p2);
    Eigen::Vector3f p3;
    fillVec(mesh.vertices[triangle[2]], p3);
    Eigen::Vector3f n = ((p2 - p1).cross(p3 - p1)).normalized();
    updateNormal(n, triangle[0], normals);
    updateNormal(n, triangle[1], normals);
    updateNormal(n, triangle[2], normals);
  }

  bool has_colors = mesh.vertices.size() == mesh.vertex_colors.size();
  for (size_t i = 0; i < mesh.vertices.size(); ++i) {
    const auto& p = mesh.vertices[i];
    mesh_->position(p.x, p.y, p.z);
    const Eigen::Vector4f n = normals.block<4, 1>(0, i);
    if (n[3] == 0.0f) {
      // not touched by any faces so default normal doesn't matter
      mesh_->normal(0.0, 0.0, 1.0);
    } else {
      mesh_->normal(n.x() / n[3], n.y() / n[3], n.z() / n[3]);
    }

    if (has_colors) {
      const auto& c = mesh.vertex_colors[i];
      mesh_->colour(c.r, c.g, c.b, c.a);
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
  ROS_ASSERT_MSG(material, "invalid material @ '%s'", material_name_.c_str());
  return *material;
}

void MeshVisual::setCullMode() {
  auto& material = getMaterial();
  ROS_DEBUG_STREAM("Setting culling for " << material_name_ << ": " << std::boolalpha
                                          << cull_faces_);
  material.setCullingMode(cull_faces_ ? Ogre::CULL_CLOCKWISE : Ogre::CULL_NONE);
}

void MeshVisual::setLightingMode() {
  auto& material = getMaterial();
  ROS_DEBUG_STREAM("Setting lighting for " << material_name_ << ": " << std::boolalpha
                                           << lighting_enabled_);

  if (lighting_enabled_) {
    material.getTechnique(0)->setLightingEnabled(true);
    const auto pass = material.getTechnique(0)->getPass(0);
    pass->setVertexColourTracking(Ogre::TVC_AMBIENT | Ogre::TVC_EMISSIVE |
                                  Ogre::TVC_DIFFUSE | Ogre::TVC_SPECULAR);
  } else {
    material.getTechnique(0)->setLightingEnabled(false);
    const auto pass = material.getTechnique(0)->getPass(0);
    pass->setVertexColourTracking(Ogre::TVC_NONE);
  }
}

}  // namespace kimera_pgmo
