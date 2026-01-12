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
namespace {

struct TestMesh {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  std::vector<traits::Timestamp> stamps;
  std::vector<traits::Face> faces;

  void addVertex(size_t offset) {
    auto& p = vertices.emplace_back();
    p.x = 1.0 + offset;
    p.y = 2.0 + offset;
    p.z = 3.0 + offset;
    p.r = 4 + static_cast<uint8_t>(offset);
    p.r = 5 + static_cast<uint8_t>(offset);
    p.r = 6 + static_cast<uint8_t>(offset);
    p.r = 7 + static_cast<uint8_t>(offset);
    stamps.push_back(8 + offset);
  }
};

traits::VertexTraits makeTraits(uint8_t r, uint8_t g, uint8_t b) {
  traits::VertexTraits traits;
  traits.properties.has_color = true;
  traits.color = {r, g, b, 255};
  return traits;
}

void addVertexToDelta(MeshDelta& delta, size_t offset) {
  traits::VertexTraits traits;
  traits.properties.has_color = true;
  traits.properties.has_stamp = true;
  traits.color = {static_cast<uint8_t>(4 + offset),
                  static_cast<uint8_t>(5 + offset),
                  static_cast<uint8_t>(6 + offset),
                  static_cast<uint8_t>(7 + offset)};
  traits.stamp = 8 + offset;
  delta.addVertex(traits::Pos(1.0 + offset, 2.0 + offset, 3.0 + offset), traits);
}

std::vector<size_t> getOffsets(const pcl::PointCloud<pcl::PointXYZRGBA>& cloud) {
  std::vector<size_t> offsets;
  for (const auto& p : cloud) {
    offsets.push_back(static_cast<size_t>(p.x - 1.0));
  }

  return offsets;
}

}  // namespace

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

TEST(MeshDelta, UpdateSimple) {
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
  MeshDelta delta2(info);
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

TEST(MeshDelta, UpdateWithPendingFaces) {
  TestMesh result;
  result.addVertex(0);
  result.addVertex(1);
  result.addVertex(2);
  result.addVertex(3);
  result.faces.push_back({0, 1, 2});
  result.faces.push_back({1, 2, 3});

  // one face archived, one face pending
  MeshOffsetInfo offsets{3, 0, 1};
  StampedCloud<pcl::PointXYZRGBA> vertex_wrapper(result.vertices, result.stamps);

  {  // add a new face that will be archived
    const auto info = MeshDelta::TrackingInfo::with_remap(0, 1, 0, {{0, 3}});
    MeshDelta delta(info);
    addVertexToDelta(delta, 4);
    addVertexToDelta(delta, 5);
    addVertexToDelta(delta, 6);
    addVertexToDelta(delta, 3);
    delta.addFace({0, 1, 2});
    delta.updateMesh(vertex_wrapper, result.faces, offsets);
  }

  {  // test scope
    EXPECT_EQ(result.stamps.size(), result.vertices.size());
    EXPECT_EQ(result.vertices.size(), 7u);
    EXPECT_EQ(result.faces.size(), 3u);

    const std::vector<size_t> expected_offsets{0, 1, 2, 4, 5, 6, 3};
    EXPECT_EQ(getOffsets(result.vertices), expected_offsets);
    const std::vector<traits::Face> expected_faces{{0, 1, 2}, {1, 2, 6}, {3, 4, 5}};
    EXPECT_EQ(result.faces, expected_faces);
  }

  {  // archive everything but 3
    const auto info = MeshDelta::TrackingInfo::with_remap(0, 1, 0, {{0, 1}});
    MeshDelta delta(info);
    addVertexToDelta(delta, 7);
    addVertexToDelta(delta, 3);
    addVertexToDelta(delta, 8);
    addVertexToDelta(delta, 9);
    delta.addFace({1, 2, 3});
    delta.addFace({0, 1, 2});
    delta.updateMesh(vertex_wrapper, result.faces, offsets);
  }

  {  // test scope
    EXPECT_EQ(result.stamps.size(), result.vertices.size());
    EXPECT_EQ(result.vertices.size(), 10u);
    EXPECT_EQ(result.faces.size(), 5u);

    const std::vector<size_t> expected_offsets{0, 1, 2, 4, 5, 6, 7, 3, 8, 9};
    EXPECT_EQ(getOffsets(result.vertices), expected_offsets);
    const std::vector<traits::Face> expected_faces{
        {0, 1, 2}, {3, 4, 5}, {1, 2, 7}, {7, 8, 9}, {6, 7, 8}};
    EXPECT_EQ(result.faces, expected_faces);
  }
}

}  // namespace kimera_pgmo
