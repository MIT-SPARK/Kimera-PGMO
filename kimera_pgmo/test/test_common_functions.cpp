/**
 * @file   test_common_functions.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>

#include "kimera_pgmo/utils/common_functions.h"
#include "kimera_pgmo/utils/common_structs.h"
#include "kimera_pgmo/utils/mesh_io.h"
#include "test_config.h"

namespace kimera_pgmo {

TEST(TestCommonFunctions, polygonsEqual) {
  pcl::Vertices p0, p1, p2, p3;

  p0.vertices.push_back(0);
  p0.vertices.push_back(1);
  p0.vertices.push_back(2);

  p1.vertices.push_back(2);
  p1.vertices.push_back(0);
  p1.vertices.push_back(1);

  p2.vertices.push_back(0);
  p2.vertices.push_back(1);
  p2.vertices.push_back(3);

  p3.vertices.push_back(0);
  p3.vertices.push_back(1);
  p3.vertices.push_back(2);
  p3.vertices.push_back(4);

  EXPECT_TRUE(PolygonsEqual(p0, p0));
  EXPECT_TRUE(PolygonsEqual(p0, p1));
  EXPECT_FALSE(PolygonsEqual(p0, p2));
  EXPECT_FALSE(PolygonsEqual(p0, p3));
}

TEST(TestCommonFunctions, pclPointToGtsam) {
  pcl::PointXYZRGBA point_rgb;
  point_rgb.x = 1.0;
  point_rgb.y = 2.0;
  point_rgb.z = -0.3;
  point_rgb.r = 22;
  point_rgb.g = 132;
  point_rgb.b = 255;
  point_rgb.a = 255;
  gtsam::Point3 gtsam_rgb_pt = PclToGtsam<pcl::PointXYZRGBA>(point_rgb);

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(point_rgb.x, point_rgb.y, point_rgb.z),
                                  gtsam_rgb_pt));

  pcl::PointXYZ point;
  point.x = -1.0;
  point.y = 20.0;
  point.z = 0.35;
  gtsam::Point3 gtsam_pt = PclToGtsam<pcl::PointXYZ>(point);

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(point.x, point.y, point.z), gtsam_pt));
}

// Combine Meshes
TEST(TestCommonFunctions, combineMeshesNoCheck) {
  pcl::PolygonMeshPtr sphere_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", sphere_mesh);

  // Test combining sphere mesh with itself and not check for duplicates
  pcl::PolygonMesh combined = CombineMeshes(*sphere_mesh, *sphere_mesh, false);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(combined.cloud, cloud);

  pcl::PointXYZ first_point(0, 0, -127);
  pcl::PointXYZ last_point(0, 0, 127);

  // Check number of surfaces
  EXPECT_EQ(size_t(1680), combined.polygons.size());
  EXPECT_EQ(size_t(844), cloud.points.size());

  // Check polygons parsed correctly
  EXPECT_EQ(0, combined.polygons[0].vertices[0]);
  EXPECT_EQ(2, combined.polygons[0].vertices[2]);
  EXPECT_EQ(842, combined.polygons[1679].vertices[0]);
  EXPECT_EQ(813, combined.polygons[1679].vertices[2]);

  // Check vertices parsed correctly
  EXPECT_EQ(-127, cloud.points[0].z);
  EXPECT_EQ(127, cloud.points[843].z);
}

TEST(TestCommonFunctions, combineMeshesCheck) {
  pcl::PolygonMeshPtr sphere_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", sphere_mesh);

  // Test combining sphere mesh with itself and check for duplicates
  pcl::PolygonMesh combined = CombineMeshes(*sphere_mesh, *sphere_mesh, true);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(combined.cloud, cloud);

  pcl::PointXYZ first_point(0, 0, -127);
  pcl::PointXYZ last_point(0, 0, 127);

  // Check number of surfaces
  EXPECT_EQ(size_t(840), combined.polygons.size());
  EXPECT_EQ(size_t(422), cloud.points.size());

  // Check polygons parsed correctly
  EXPECT_EQ(0, combined.polygons[0].vertices[0]);
  EXPECT_EQ(2, combined.polygons[0].vertices[2]);
  EXPECT_EQ(420, combined.polygons[839].vertices[0]);
  EXPECT_EQ(391, combined.polygons[839].vertices[2]);

  // Check vertices parsed correctly
  EXPECT_EQ(-127, cloud.points[0].z);
  EXPECT_EQ(127, cloud.points[421].z);
}

TEST(TestCommonFunctions, combineMeshesIndices) {
  pcl::PolygonMeshPtr sphere_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", sphere_mesh);

  // Test combining sphere with itself, check for duplicates with given vertices
  std::vector<size_t> ind_to_check{0, 1, 2, 3, 4, 5, 6, 100, 200, 300, 421};
  std::vector<size_t> new_indices;
  pcl::PolygonMesh combined =
      CombineMeshes(*sphere_mesh, *sphere_mesh, ind_to_check, &new_indices);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(combined.cloud, cloud);

  pcl::PointXYZ first_point(0, 0, -127);
  pcl::PointXYZ last_point(-1, 26, 124);

  // Check number of surfaces
  EXPECT_EQ(size_t(1675), combined.polygons.size());
  EXPECT_EQ(size_t(833), cloud.points.size());

  // Check polygons parsed correctly
  EXPECT_EQ(0, combined.polygons[0].vertices[0]);
  EXPECT_EQ(2, combined.polygons[0].vertices[2]);
  EXPECT_EQ(832, combined.polygons[1674].vertices[0]);
  EXPECT_EQ(803, combined.polygons[1674].vertices[2]);

  // Check vertices parsed correctly
  EXPECT_EQ(-127, cloud.points[0].z);
  EXPECT_EQ(124, cloud.points[832].z);

  // Check the indices of added mesh
  EXPECT_EQ(size_t(422), new_indices.size());
  EXPECT_EQ(0, new_indices[0]);
  EXPECT_EQ(422, new_indices[7]);
  EXPECT_EQ(832, new_indices[420]);
  EXPECT_EQ(421, new_indices[421]);
}

TEST(TestCommonFunctions, polygonEquality) {
  pcl::Vertices poly_1, poly_2, poly_3, poly_4;

  poly_1.vertices = std::vector<uint32_t>{0, 1, 2};
  poly_2.vertices = std::vector<uint32_t>{2, 0, 1};
  poly_3.vertices = std::vector<uint32_t>{1, 0, 2};
  poly_4.vertices = std::vector<uint32_t>{3, 4, 5};

  EXPECT_TRUE(PolygonsEqual(poly_1, poly_1));
  EXPECT_TRUE(PolygonsEqual(poly_1, poly_2));
  EXPECT_FALSE(PolygonsEqual(poly_1, poly_3));
  EXPECT_FALSE(PolygonsEqual(poly_1, poly_4));
  EXPECT_TRUE(PolygonsEqual(poly_4, poly_4));
}

TEST(TestCommonFunctions, meshSurfaceExist) {
  std::map<size_t, std::vector<size_t> > adj_surfaces;

  pcl::Vertices poly_1, poly_2, poly_3, poly_4;
  poly_1.vertices = std::vector<uint32_t>{0, 1, 2};
  poly_2.vertices = std::vector<uint32_t>{0, 2, 3};
  poly_3.vertices = std::vector<uint32_t>{0, 5, 1};
  poly_4.vertices = std::vector<uint32_t>{1, 4, 2};
  std::vector<pcl::Vertices> surfaces{poly_1, poly_2, poly_3, poly_4};

  adj_surfaces[0] = std::vector<size_t>{0, 1};
  adj_surfaces[1] = std::vector<size_t>{0, 3};
  adj_surfaces[2] = std::vector<size_t>{0, 1, 3};
  adj_surfaces[3] = std::vector<size_t>{1};
  adj_surfaces[4] = std::vector<size_t>{3};
  EXPECT_FALSE(SurfaceExists(poly_3, adj_surfaces, surfaces));

  adj_surfaces[0].push_back(2);
  adj_surfaces[1].push_back(2);
  adj_surfaces[5] = std::vector<size_t>{2};
  EXPECT_TRUE(SurfaceExists(poly_3, adj_surfaces, surfaces));
}

}  // namespace kimera_pgmo
