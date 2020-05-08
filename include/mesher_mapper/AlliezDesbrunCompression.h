/**
 * @file   AlliezDesbrunCompression.h
 * @brief  Simplify and reconstruct meshes Alliez Desbrun 2001
 * @author Yun Chang
 */
#include "mesher_mapper/MeshCompression.h"

namespace mesher_mapper {

enum class VertexStatus {
  UNCONQUERED = 0; CONQUERED = 1; REMOVED = 2;  // To be removed
}
class AlliezDesbrunCompression {
 public:
  AlliezDesbrunCompression();
  ~AlliezDesbrunCompression();

  bool setInputMesh(pcl::PolygonMeshPtr input_mesh);
  bool process();

 private:
  size_t level_of_detail_;

  std::map<Vertex, VertexStatus> vertex_status_; 
  std::map<Vertex, Triangle> vertex_to_triangle_;
  
  void DecimatingConquest(pcl::PolygonMeshPtr output_mesh,
                          GraphPtr ouput_graph);
};
}  // namespace mesher_mapper