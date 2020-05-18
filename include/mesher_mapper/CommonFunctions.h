/**
 * @file   CommonFunctions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl_msgs/PolygonMesh.h>

namespace mesher_mapper {

void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh);

pcl_msgs::PolygonMesh ConstructPolygonMeshMsg(const pcl::PolygonMesh& mesh,
                                              const std_msgs::Header& header);

}  // namespace mesher_mapper
