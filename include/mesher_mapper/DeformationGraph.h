/**
 * @file   DeformationGraph.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */

#include <map>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <pcl/PolygonMesh.h>

namespace mesher_mapper {
class DeformationGraph {
 public:
  DeformationGraph();
  DeformationGraph(pcl::PolygonMesh mesh);
  ~DeformationGraph();

 private:
  std::map<size_t, gtsam::Pose3> keyed_tranforms_;
  std::map<size_t, std::vector<size_t>> connections_;
};
}  // namespace mesher_mapper