/**
 * @file   DeformationSolver.h
 * @brief  Deforms mesh based on lates optimized trajectory
 * @author Yun Chang
 */

#include <pcl/surface/simplification_remove_unused_vertices.h>

using pcl::PolygonMesh;

namespace mesher_mapper {

DeformationGraph::DeformationGraph() {}
DeformationGraph::DeformationGraph(pcl::PolygonMesh mesh) {

  // Simplify mesh 
  PolygonMesh simplified_mesh;
  std::vector<int> simplified_indices;
  pcl::surface::SimplificationRemoveUnusedVertices::simplify(
      mesh, simplified_mesh, simplified_indices);
  // Add to object 
  for (int idx : simplified_indices) {
  	keyed_transforms_[idx] = gtsam::Pose3();
  }
}
DeformationGraph::~DeformationGraph();


}  // namespace mesher_mapper