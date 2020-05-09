/**
 * @file   MeshCompression.h
 * @brief  Simplify and reconstruct meshes
 * @author Yun Chang
 */
#include <map>

#include <pcl/PolygonMesh.h>
#include <Eigen/Dense>

#include "mesher_mapper/CommonStructs.h"

namespace mesher_mapper {

typedef Eigen::Vector3d Pointxyz; 

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
  pcl::PolygonMesh original_mesh_;
  pcl::PolygonMesh base_mesh_;
  std::map<Vertex, Pointxyz> vertex_positions_;
};
}  // namespace mesher_mapper