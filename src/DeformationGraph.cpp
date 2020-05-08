/**
 * @file   DeformationSolver.h
 * @brief  Deforms mesh based on lates optimized trajectory
 * @author Yun Chang
 */

#include <pcl/surface/vtk_smoothing/vtk_mesh_quadric_decimation.h>

#include "mesher_mapper/DeformationGraph.h"

using pcl::PolygonMesh;

namespace mesher_mapper {

DeformationGraph::DeformationGraph() {}
DeformationGraph::DeformationGraph(const pcl::PolygonMeshConstPtr& mesh) {
  // Simplify mesh
  PolygonMesh simplified_mesh;
  std::vector<int> simplified_indices;
  pcl::MeshQuadricDecimationVTK mesh_decimation;
  mesh_decimation.setInputMesh(mesh);
  // mesh_decimation.setTargetReductionFactor(1000.0);
  mesh_decimation.process(simplified_mesh);
  std::cout << "simplified mesh size: " << simplified_mesh.cloud.data.size()
            << "with reduction factor: "
            << mesh_decimation.getTargetReductionFactor() << std::endl;
  for (pcl::Vertices polygon : mesh->polygons) {
    std::cout << "polygon ";
    for (int idx : polygon.vertices) {
      std::cout << idx << " ";
    }
    std::cout << "\n";
  }
  // Add to object
  // for (int idx : simplified_indices) {
  //   keyed_transforms_.insert(
  //       std::map<size_t, gtsam::Pose3>::value_type(idx, gtsam::Pose3()));
  //   connections_[idx] = std::vector<size_t>();
  // }
  // // Add to connections by reaading the polygon message
  // // Here we assume that we are dealing with a triangular mesh (TODO: double
  // // check this)
  // for (pcl::Vertices polygon : simplified_mesh.polygons) {
  //   for (int idx : polygon.vertices) {
  //     for (int idx_to : polygon.vertices) {
  //       if (idx != idx_to) {
  //         std::vector<size_t>::iterator it =
  //             find(connections_[idx].begin(), connections_[idx].end(),
  //             idx_to);
  //         if (it == connections_[idx].end()) {
  //           connections_[idx].push_back(idx_to);
  //         }
  //       }
  //     }
  //   }
  // }
}

DeformationGraph::~DeformationGraph() {}

}  // namespace mesher_mapper