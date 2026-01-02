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
  std::vector<traits::Timestamp> stamps;
  std::vector<traits::Face> faces;
};

traits::VertexTraits makeTraits(uint8_t r, uint8_t g, uint8_t b) {
  traits::VertexTraits traits;
  traits.properties.has_color = true;
  traits.color = {r, g, b, 255};
  return traits;
}

TEST(MeshDelta, AddVertexCorrect) {
  MeshDelta delta({0, 0, 0});

  EXPECT_EQ(delta.addVertex(traits::Pos(1.0, 0.0, 0.0), {}, true), 0u);
  EXPECT_EQ(delta.getNumVertices(), 1u);
  EXPECT_EQ(delta.getNumActiveVertices(), 0u);
  EXPECT_EQ(delta.getNumArchivedVertices(), 1u);

  EXPECT_EQ(delta.addVertex(traits::Pos(2.0, 0.0, 0.0), {}, true), 1u);
  EXPECT_EQ(delta.getNumVertices(), 2u);
  EXPECT_EQ(delta.getNumActiveVertices(), 0u);
  EXPECT_EQ(delta.getNumArchivedVertices(), 2u);

  EXPECT_EQ(delta.addVertex(traits::Pos(3.0, 0.0, 0.0), {}, false), 2u);
  EXPECT_EQ(delta.getNumVertices(), 3u);
  EXPECT_EQ(delta.getNumActiveVertices(), 1u);
  EXPECT_EQ(delta.getNumArchivedVertices(), 2u);

  EXPECT_EQ(delta.addVertex(traits::Pos(4.0, 0.0, 0.0), {}), 3u);
  EXPECT_EQ(delta.getNumVertices(), 4u);
  EXPECT_EQ(delta.getNumActiveVertices(), 2u);
  EXPECT_EQ(delta.getNumArchivedVertices(), 2u);

  for (size_t i = 0; i < 4; ++i) {
    EXPECT_EQ(delta.getVertex(i).pos.x(), i + 1);
  }

  traits::VertexProperties expected_props{false, false, false, false};
  EXPECT_EQ(delta.vertex_properties(), expected_props);

  traits::Pos p(5.0, 0.0, 0.0);
  const auto expected = makeTraits(1, 2, 3);
  delta.addVertex(p, expected, false);

  traits::VertexTraits result;
  EXPECT_EQ(traits::get_vertex(delta, 4, &result), p);
  EXPECT_EQ(result, expected);

  expected_props = {true, false, false, false};
  EXPECT_EQ(traits::get_vertex_properties(delta), expected_props);
}

TEST(MeshDelta, AddFacesCorrect) {
  MeshDelta delta({0, 0, 0});

  delta.addFace({1, 2, 3}, false);
  EXPECT_EQ(delta.getNumFaces(), 1u);
  EXPECT_EQ(delta.getNumActiveFaces(), 1u);
  EXPECT_EQ(delta.getNumArchivedFaces(), 0u);

  delta.addFace({2, 3, 4}, false);
  EXPECT_EQ(delta.getNumFaces(), 2u);
  EXPECT_EQ(delta.getNumActiveFaces(), 2u);
  EXPECT_EQ(delta.getNumArchivedFaces(), 0u);

  delta.addFace({3, 4, 5}, true);
  EXPECT_EQ(delta.getNumFaces(), 3u);
  EXPECT_EQ(delta.getNumActiveFaces(), 2u);
  EXPECT_EQ(delta.getNumArchivedFaces(), 1u);

  delta.addFace({4, 5, 6}, true);
  EXPECT_EQ(delta.getNumFaces(), 4u);
  EXPECT_EQ(delta.getNumActiveFaces(), 2u);
  EXPECT_EQ(delta.getNumArchivedFaces(), 2u);

  EXPECT_EQ(delta.getFace(0)[0], 3u);
  EXPECT_EQ(delta.getFace(1)[0], 4u);
  EXPECT_EQ(delta.getFace(2)[0], 1u);
  EXPECT_EQ(delta.getFace(3)[0], 2u);
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

  MeshOffsetInfo offsets;
  delta1.updateMesh(vertex_wrapper, result.faces, offsets);

  {  // test scope
    EXPECT_EQ(result.stamps.size(), result.vertices.size());
    EXPECT_EQ(result.vertices.size(), 4u);
    for (size_t i = 0; i < result.vertices.size(); ++i) {
      EXPECT_EQ(static_cast<size_t>(result.vertices[i].r), i);
    }

    const std::vector<traits::Face> expected_faces{{0, 1, 2}, {1, 2, 3}};
    EXPECT_EQ(result.faces, expected_faces);
  }

  // Fake archival of 2 vertices and 1 face
  const auto info = MeshDelta::TrackingInfo::with_remap(1, 2, 1, {{0, 0}, {1, 1}});
  MeshDelta delta2({1, 2, 1});
  delta2.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(2, 0, 0));
  delta2.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(3, 0, 0));
  delta2.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(4, 0, 0));
  delta2.addVertex(traits::Pos(1.0, 2.0, 3.0), makeTraits(5, 0, 0));
  delta2.addFace({0, 1, 2});
  delta2.addFace({1, 2, 3});

  delta2.updateMesh(vertex_wrapper, result.faces, offsets);

  {  // test scope
    EXPECT_EQ(result.stamps.size(), result.vertices.size());
    EXPECT_EQ(result.vertices.size(), 6u);
    EXPECT_EQ(result.faces.size(), 3u);
    for (size_t i = 0; i < result.vertices.size(); ++i) {
      EXPECT_EQ(static_cast<size_t>(result.vertices[i].r), i);
    }

    const std::vector<traits::Face> expected_faces{{0, 1, 2}, {2, 3, 4}, {3, 4, 5}};
    EXPECT_EQ(result.faces, expected_faces);
  }
}

}  // namespace kimera_pgmo
