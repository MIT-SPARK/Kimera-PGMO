/**
 * @file   CommonFunctions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */
#include <pcl/PCLPointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include "mesher_mapper/CommonFunctions.h"

namespace mesher_mapper {

void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh) {
  std::cout << "filename: " << filename << std::endl;
  pcl::PLYReader ply_reader;
  ply_reader.read(filename, *mesh);
}

pcl_msgs::PolygonMesh ConstructPolygonMeshMsg(const pcl::PolygonMesh& mesh,
                                              const std_msgs::Header& header) {
  pcl_msgs::PolygonMesh new_msg;
  pcl_conversions::fromPCL(mesh.cloud, new_msg.cloud);
  for (pcl::Vertices polygon : mesh.polygons) {
    pcl_msgs::Vertices polygon_msg;
    for (size_t i : polygon.vertices) {
      polygon_msg.vertices.push_back(i);
    }
    new_msg.polygons.push_back(polygon_msg);
  }
  new_msg.header = header;
}

}  // namespace mesher_mapper