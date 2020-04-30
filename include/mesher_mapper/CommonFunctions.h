/**
 * @file   CommonFunctions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <string>

#include <pcl/PolygonMesh.h>

using pcl::PolygonMesh;

namespace mesher_mapper {

void ReadMeshFromPly(const std::string& filename, PolygonMesh* mesh);

}  // namespace mesher_mapper
