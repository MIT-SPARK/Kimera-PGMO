/**
 * @file   test_mesh_io.cpp
 * @brief  Test reading and writing from PLY files
 * @author Yun Chang
 */

#include <gtest/gtest.h>
#include <pcl/conversions.h>

#include <cstdlib>
#include <filesystem>
#include <fstream>

#include "kimera_pgmo/utils/mesh_io.h"
#include "test_config.h"

namespace kimera_pgmo {

class MeshIOExport : public testing::Test {
 protected:
  void SetUp() override {
    auto pattern =
        (std::filesystem::temp_directory_path() / "kimera_pgmo_ply_XXXXXX").string();
    ASSERT_NE(mkdtemp(pattern.data()), nullptr);
    directory_ = pattern;
  }

  void TearDown() override {
    if (directory_.empty()) {
      return;
    }

    std::filesystem::remove_all(directory_);
  }

  std::filesystem::path directory_;
};

TEST_F(MeshIOExport, legacySaveDefaultsToAscii) {
  IOData data;
  data.x = {1.25f};
  data.y = {-2.5f};
  data.z = {3.75f};
  const auto filename = (directory_ / "legacy.ply").string();
  data.save(filename);

  std::ifstream stream(filename, std::ios::binary);
  std::string line;
  ASSERT_TRUE(std::getline(stream, line));
  EXPECT_EQ(line, "ply");
  ASSERT_TRUE(std::getline(stream, line));
  EXPECT_EQ(line, "format ascii 1.0");

  const auto restored = IOData::load(filename);
  ASSERT_NE(restored, nullptr);
  EXPECT_EQ(restored->x, data.x);
  EXPECT_EQ(restored->y, data.y);
  EXPECT_EQ(restored->z, data.z);
}

TEST_F(MeshIOExport, formatsPreserveVertexOrderAndAttributes) {
  const std::vector<traits::Vertex> vertices{{{1.25f, -2.5f, 0.0f},
                                              {{true, true, true, true},
                                               {255, 0, 10, 17},
                                               1700000000123456789ULL,
                                               4000000001U,
                                               1699999999987654321ULL}},
                                             {{0.0f, 3.75f, 0.125f},
                                              {{true, true, true, true},
                                               {30, 128, 255, 99},
                                               1700000001987654321ULL,
                                               256U,
                                               1700000000000000001ULL}},
                                             {{-4.5f, 0.25f, 2.0f},
                                              {{true, true, true, true},
                                               {0, 255, 100, 255},
                                               1700000002000000007ULL,
                                               0U,
                                               1700000001000000003ULL}}};
  const std::vector<traits::Face> faces{{2, 0, 1}};

  for (const auto binary : {false, true}) {
    SCOPED_TRACE(binary ? "binary" : "default ASCII");
    const auto filename = (directory_ / (binary ? "binary.ply" : "ascii.ply")).string();
    if (binary) {
      WriteMesh(filename, vertices, faces, true);
    } else {
      WriteMesh(filename, vertices, faces);
    }

    std::ifstream stream(filename, std::ios::binary);
    std::string line;
    ASSERT_TRUE(std::getline(stream, line));
    EXPECT_EQ(line, "ply");
    ASSERT_TRUE(std::getline(stream, line));
    EXPECT_EQ(line, binary ? "format binary_little_endian 1.0" : "format ascii 1.0");

    std::vector<traits::Vertex> restored_vertices;
    std::vector<traits::Face> restored_faces;
    ReadMesh(filename, restored_vertices, restored_faces);
    ASSERT_EQ(restored_vertices.size(), vertices.size());
    EXPECT_EQ(restored_faces, faces);
    for (size_t i = 0; i < vertices.size(); ++i) {
      EXPECT_EQ(restored_vertices[i].pos, vertices[i].pos);
      EXPECT_EQ(restored_vertices[i].traits, vertices[i].traits);
    }
  }
}

TEST(TestMeshIO, readWritePly) {
  pcl::PolygonMeshPtr original_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/cube.ply", original_mesh);
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromPCLPointCloud2(original_mesh->cloud, cloud);

  std::vector<Timestamp> vertex_stamps;
  for (size_t i = 0; i < cloud.size(); i++) {
    vertex_stamps.push_back(i);
  }

  // Write mesh with stamp to ply
  WriteMeshWithStampsToPly(
      std::string(DATASET_PATH) + "/cube.ply", *original_mesh, vertex_stamps);

  // Then read again
  pcl::PolygonMeshPtr read_mesh(new pcl::PolygonMesh());
  std::vector<Timestamp> read_stamps;
  ReadMeshWithStampsFromPly(
      std::string(DATASET_PATH) + "/cube.ply", read_mesh, &read_stamps);

  pcl::PointCloud<pcl::PointXYZRGBA> read_cloud;
  pcl::fromPCLPointCloud2(read_mesh->cloud, read_cloud);

  ASSERT_EQ(read_cloud.size(), cloud.size());
  for (size_t i = 0; i < cloud.size(); i++) {
    EXPECT_EQ(cloud.points[i].x, read_cloud.points[i].x);
    EXPECT_EQ(cloud.points[i].y, read_cloud.points[i].y);
    EXPECT_EQ(cloud.points[i].z, read_cloud.points[i].z);
    EXPECT_EQ(cloud.points[i].r, read_cloud.points[i].r);
    EXPECT_EQ(cloud.points[i].g, read_cloud.points[i].g);
    EXPECT_EQ(cloud.points[i].b, read_cloud.points[i].b);
    EXPECT_EQ(cloud.points[i].a, read_cloud.points[i].a);
    EXPECT_EQ(vertex_stamps[i], read_stamps[i]);
  }

  ASSERT_EQ(original_mesh->polygons.size(), read_mesh->polygons.size());
  for (size_t i = 0; i < original_mesh->polygons.size(); i++) {
    for (size_t j = 0; j < 3; j++) {
      EXPECT_EQ(original_mesh->polygons[i].vertices[j],
                read_mesh->polygons[i].vertices[j]);
    }
  }

  // Finally write original ply back
  WriteMeshToPly(std::string(DATASET_PATH) + "/cube.ply", *original_mesh);
}

}  // namespace kimera_pgmo
