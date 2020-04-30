/**
 * @file   CommonFunctions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#include <pcl/io/ply_io.h>

#include "mesher_mapper/CommonFunctions.h"

namespace mesher_mapper {

void ReadMeshFromPly(const std::string& filename, PolygonMesh* mesh) {
  pcl::PLYReader ply_reader;
  ply_reader.read(filename, *mesh);
}

}  // namespace mesher_mapper