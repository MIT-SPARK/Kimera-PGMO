/**
 * @file   testCommonFunctions.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include "gtest/gtest.h"

#include <pcl/PolygonMesh.h>

#include "mesher_mapper/CommonFunctions.h"
#include "test_config.h"

namespace mesher_mapper {
TEST(CommonFunctions, testReadPLY) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", mesh);

  EXPECT_EQ(size_t(840), mesh->polygons.size());
}
}  // namespace mesher_mapper

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
