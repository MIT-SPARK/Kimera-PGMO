/**
 * @file   CommonFunctions.cpp
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#include "mesher_mapper/CommonFunctions.h"

namespace mesher_mapper {

void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh) {
  std::cout << "filename: " << filename << std::endl;
  pcl::PLYReader ply_reader;
  ply_reader.read(filename, *mesh);
}

}  // namespace mesher_mapper