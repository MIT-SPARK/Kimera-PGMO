/*
 * @file   test_mesh_delta.cpp
 * @brief  Unit-tests for MeshDelta
 * @author Nathan Hughes
 */
#include <gtest/gtest.h>

#include <cstdint>

#include <pcl/impl/point_types.hpp>

#include "kimera_pgmo/mesh_delta.h"
#include "kimera_pgmo/mesh_types.h"
#include "kimera_pgmo/pcl_mesh_traits.h"

namespace kimera_pgmo {

struct TestMesh {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  std::vector<pcl::Vertices> faces;
  std::vector<traits::Timestamp> stamps;
};

traits::VertexTraits makeTraits(uint8_t r, uint8_t g, uint8_t b) {
  traits::VertexTraits traits;
  traits.properties.has_color = true;
  traits.color = {r, g, b, 255};
  return traits;
}

TEST(MeshDelta, updateSimple) {
  MeshDelta delta1({0, 0, 0});
  delta1.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(0, 0, 0));
  delta1.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(1, 0, 0));
  delta1.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(2, 0, 0));
  delta1.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(3, 0, 0));
  delta1.addFace({0, 1, 2});
  delta1.addFace({1, 2, 3});

  TestMesh result;
  StampedCloud<pcl::PointXYZRGBA> vertex_wrapper(result.vertices, result.stamps);
  delta1.updateMesh(vertex_wrapper, result.faces);
  EXPECT_EQ(result.stamps.size(), result.vertices.size());
  EXPECT_EQ(result.vertices.size(), 4u);
  EXPECT_EQ(result.faces.size(), 2u);
  for (size_t i = 0; i < result.vertices.size(); ++i) {
    EXPECT_EQ(static_cast<size_t>(result.vertices[i].r), i);
  }

  // Fake archival of 2 vertices and 1 face
  MeshDelta delta2({1, 2, 1});
  delta2.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(2, 0, 0));
  delta2.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(3, 0, 0));
  delta2.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(4, 0, 0));
  delta2.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(5, 0, 0));
  delta2.addFace({0, 1, 2});
  delta2.addFace({1, 2, 3});

  delta2.updateMesh(vertex_wrapper, result.faces);
  EXPECT_EQ(result.stamps.size(), result.vertices.size());
  EXPECT_EQ(result.vertices.size(), 6u);
  EXPECT_EQ(result.faces.size(), 3u);
  for (size_t i = 0; i < result.vertices.size(); ++i) {
    EXPECT_EQ(static_cast<size_t>(result.vertices[i].r), i);
  }
}

TEST(MeshDelta, archiveVerticesCorrect) {
  MeshDelta delta({0, 0, 0});
  EXPECT_EQ(delta.addVertex(traits::Pos(1.0, 2.0, 3.0), {}, true), 0u);
  EXPECT_EQ(delta.addVertex(traits::Pos(1.0, 2.0, 3.0), {}, true), 1u);
  EXPECT_EQ(delta.addVertex(traits::Pos(1.0, 2.0, 3.0), {}, false), 2u);
  EXPECT_EQ(delta.addVertex(traits::Pos(1.0, 2.0, 3.0), {}), 3u);
  EXPECT_EQ(delta.getNumArchivedVertices(), 2u);
}

TEST(MeshDelta, archiveFacesCorrect) {
  MeshDelta delta({0, 0, 0});
  delta.addFace({0, 1, 2}, true);
  delta.addFace({1, 2, 3}, true);
  delta.addFace({0, 1, 2}, false);
  delta.addFace({1, 2, 3}, false);
  EXPECT_EQ(delta.getNumArchivedFaces(), 2u);
}

}  // namespace kimera_pgmo
