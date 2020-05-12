/**
 * @file   CommonFunctions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/PolygonMesh.h>

namespace mesher_mapper {

void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh);

}  // namespace mesher_mapper
