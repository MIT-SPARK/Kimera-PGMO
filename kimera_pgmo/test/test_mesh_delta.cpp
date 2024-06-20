/*
 * @file   test_mesh_delta.cpp
 * @brief  Unit-tests for MeshDelta
 * @author Nathan Hughes
 */
#include <gtest/gtest.h>

#include "kimera_pgmo/mesh_delta.h"

namespace kimera_pgmo {

struct TestMesh {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  std::vector<pcl::Vertices> faces;
  std::vector<Timestamp> stamps;
};

struct TestPoint {
  float x;
  float y;
  float z;
  int r;
  int g;
  int b;

  operator pcl::PointXYZRGBA() const {
    pcl::PointXYZRGBA point;
    point.x = x;
    point.y = y;
    point.z = z;
    point.r = r;
    point.g = g;
    point.b = b;
    point.a = 255;
    return point;
  }
};

TEST(TestMeshDelta, constructor) {
  MeshDelta mesh;
  EXPECT_EQ(mesh.vertex_start, 0u);
  EXPECT_EQ(mesh.face_start, 0u);
}

TEST(TestMeshDelta, updateSimple) {
  MeshDelta delta1;
  delta1.addVertex(100, TestPoint{1.0, 2.0, 3.0, 0, 0, 0});
  delta1.addVertex(100, TestPoint{1.0, 2.0, 3.0, 1, 0, 0});
  delta1.addVertex(100, TestPoint{1.0, 2.0, 3.0, 2, 0, 0});
  delta1.addVertex(100, TestPoint{1.0, 2.0, 3.0, 3, 0, 0});
  delta1.addFace({0, 1, 2});
  delta1.addFace({1, 2, 3});

  TestMesh result;
  delta1.updateMesh(result.vertices, result.stamps, result.faces);
  EXPECT_EQ(result.stamps.size(), result.vertices.size());
  EXPECT_EQ(result.vertices.size(), 4u);
  for (size_t i = 0; i < result.vertices.size(); ++i) {
    EXPECT_EQ(static_cast<size_t>(result.vertices[i].r), i);
  }
  EXPECT_EQ(result.faces.size(), 2u);

  MeshDelta delta2(2, 1);
  delta2.addVertex(100, TestPoint{1.0, 2.0, 3.0, 2, 0, 0});
  delta2.addVertex(100, TestPoint{1.0, 2.0, 3.0, 3, 0, 0});
  delta2.addVertex(100, TestPoint{1.0, 2.0, 3.0, 4, 0, 0});
  delta2.addVertex(100, TestPoint{1.0, 2.0, 3.0, 5, 0, 0});
  delta2.addFace({0, 1, 2});
  delta2.addFace({1, 2, 3});

  delta2.updateMesh(result.vertices, result.stamps, result.faces);
  EXPECT_EQ(result.stamps.size(), result.vertices.size());
  EXPECT_EQ(result.vertices.size(), 6u);
  for (size_t i = 0; i < result.vertices.size(); ++i) {
    EXPECT_EQ(static_cast<size_t>(result.vertices[i].r), i);
  }
  EXPECT_EQ(result.faces.size(), 3u);
}

TEST(TestMeshDelta, archiveVerticesCorrect) {
  MeshDelta delta(1, 3);
  EXPECT_EQ(delta.addVertex(100, TestPoint{1.0, 2.0, 3.0, 0, 0, 0}, std::nullopt, true),
            1u);
  EXPECT_EQ(delta.addVertex(100, TestPoint{1.0, 2.0, 3.0, 0, 0, 0}, std::nullopt, true),
            2u);
  EXPECT_EQ(
      delta.addVertex(100, TestPoint{1.0, 2.0, 3.0, 0, 0, 0}, std::nullopt, false), 3u);
  EXPECT_EQ(
      delta.addVertex(100, TestPoint{1.0, 2.0, 3.0, 0, 0, 0}, std::nullopt, false), 4u);
  EXPECT_EQ(delta.getTotalArchivedVertices(), 3u);
  EXPECT_EQ(delta.getNumArchivedVertices(), 2u);
}

TEST(TestMeshDelta, archiveFacesCorrect) {
  MeshDelta delta(1, 3);
  delta.addFace({0, 1, 2}, true);
  delta.addFace({1, 2, 3}, true);
  delta.addFace({0, 1, 2}, false);
  delta.addFace({1, 2, 3}, false);
  EXPECT_EQ(delta.getTotalArchivedFaces(), 5u);
  EXPECT_EQ(delta.getNumArchivedFaces(), 2u);
}

}  // namespace kimera_pgmo
