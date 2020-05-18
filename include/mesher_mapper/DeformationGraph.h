/**
 * @file   DeformationGraph.h
 * @brief  Deformation Graph object
 * @author Yun Chang
 */

#include <map>
#include <vector>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>

#include <pcl/PolygonMesh.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include "mesher_mapper/CommonStructs.h"

namespace mesher_mapper {

// Define a factor type for edges of deformation graph
class DeformationEdgeFactor
    : public gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3> {
 private:
  gtsam::Point3 node1_position;
  gtsam::Point3 node2_position;

 public:
  DeformationEdgeFactor(gtsam::Key node1_key,
                        gtsam::Key node2_key,
                        const gtsam::Point3& node1_point,
                        const gtsam::Point3& node2_point,
                        gtsam::SharedNoiseModel model)
      : gtsam::NoiseModelFactor2<gtsam::Pose3, gtsam::Pose3>(model,
                                                             node1_key,
                                                             node2_key),
        node1_position(node1_point),
        node2_position(node2_point) {}
  ~DeformationEdgeFactor() {}

  gtsam::Vector evaluateError(
      const gtsam::Pose3& p1,
      const gtsam::Pose3& p2,
      boost::optional<gtsam::Matrix&> H1 = boost::none,
      boost::optional<gtsam::Matrix&> H2 = boost::none) const {
    gtsam::Rot3 R1 = p1.rotation();
    gtsam::Point3 t1 = p1.translation();
    gtsam::Point3 t2_1 = t1.compose(node1_position.compose(
        R1.rotate(node1_position.between(node2_position))));
    gtsam::Point3 t2 = p2.translation();
    gtsam::Point3 t2_2 = node2_position.compose(t2);

    return t2_2.between(t2_1).vector();
  }
};

class DeformationGraph {
 public:
  DeformationGraph(const pcl::PolygonMeshConstPtr& mesh);
  ~DeformationGraph();

  void loopClose(Vertex v1,
                 Vertex v2,
                 pcl::PointCloud<pcl::PointXYZ>::Ptr new_vertex_positions);

 private:
  Graph graph_;
  pcl::PointCloud<pcl::PointXYZ> vertices_;

  gtsam::NonlinearFactorGraph consistency_factors_;
  gtsam::Values values_;
};
}  // namespace mesher_mapper