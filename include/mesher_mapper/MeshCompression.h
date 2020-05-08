/**
 * @file   MeshCompression.h
 * @brief  Simplify and reconstruct meshes
 * @author Yun Chang
 */
#include <map>

#include <gtsam/geometry/Pose3.h>
#include <pcl/PolygonMesh.h>

#include "mesher_mapper/CommonStructs.h"

namespace mesher_mapper {

class MeshCompression {
 public:
  MeshCompression() = default;
  virtual ~MeshCompression() = default;

  virtual bool process() = 0;
  virtual bool setInputMesh(pcl::PolygonMeshPtr input_mesh) = 0;
  inline void getBaseMesh(pcl::PolygonMeshPtr base_mesh) {
    *base_mesh = base_mesh_;
  }

 protected:
  Graph mesh_graph_;
  PolygonMesh original_mesh_;
  PolygonMesh base_mesh_;
  std::map<Vertex, gtsam::Pose3> vertex_positions_;
};
}  // namespace mesher_mapper