/**
 * @file   TriangleMeshConversion.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */
#include "kimera_pgmo/utils/TriangleMeshConversion.h"

#include <pcl/conversions.h>

#include "kimera_pgmo/PclMeshTraits.h"

namespace kimera_pgmo {

using ColorCloud = pcl::PointCloud<pcl::PointXYZRGBA>;
using PclFaces = std::vector<pcl::Vertices>;

mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(const pcl::PolygonMesh& mesh) {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices_cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, vertices_cloud);
  return PolygonMeshToTriangleMeshMsg(vertices_cloud, mesh.polygons);
}

mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(const ColorCloud& vertices,
                                                     const PclFaces& polygons) {
  mesh_msgs::TriangleMesh msg;
  fillTriangleMeshMsg(vertices, polygons, msg);
  return msg;
}

pcl::PolygonMesh TriangleMeshMsgToPolygonMesh(const mesh_msgs::TriangleMesh& msg) {
  pcl::PolygonMesh mesh;
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  fillFromTriangleMeshMsg(msg, cloud, mesh.polygons);
  pcl::toPCLPointCloud2(cloud, mesh.cloud);
  return mesh;
}

}  // namespace kimera_pgmo
