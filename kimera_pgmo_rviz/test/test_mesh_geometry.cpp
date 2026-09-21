#include <gtest/gtest.h>

#include "kimera_pgmo_rviz/mesh_geometry.h"

namespace kimera_pgmo {
namespace {

std::vector<traits::Face> collectFaces(const MeshGeometry& geometry, size_t cap) {
  std::vector<traits::Face> result;
  for (const auto& chunk : geometry.chunks()) {
    EXPECT_LE(chunk.vertices.size(), cap);
    EXPECT_LE(chunk.faces.size(), 2 * cap);
    for (const auto& face : chunk.faces) {
      traits::Face global;
      for (size_t j = 0; j < 3; ++j) {
        global[j] = chunk.vertices.at(face[j]);
        EXPECT_LT(global[j], geometry.vertices().size());
      }

      result.push_back(global);
    }
  }

  return result;
}

void addTriangle(MeshDelta& delta, float x, bool archived) {
  const auto start = delta.getNumVertices();
  delta.addVertex({x, 0, 0}, {}, archived);
  delta.addVertex({x + 1, 0, 0}, {}, archived);
  delta.addVertex({x, 1, 0}, {}, archived);
  delta.addFace({start, start + 1, start + 2}, archived);
}

TEST(MeshGeometry, ChunkCapsAndFullReplacement) {
  MeshGeometry geometry;
  geometry.setMaxVertices(3);
  MeshDelta first({});
  addTriangle(first, 0, false);
  addTriangle(first, 2, false);
  geometry.applyDelta(first);
  EXPECT_EQ(collectFaces(geometry, 3), geometry.faces());
  ASSERT_EQ(geometry.chunks().size(), 2u);
  geometry.setMaxVertices(6);
  EXPECT_EQ(geometry.chunks().size(), 1u);
  geometry.setMesh({}, {});
  EXPECT_TRUE(geometry.chunks().empty());
  EXPECT_EQ(geometry.applyDelta(first), 0u);
  EXPECT_EQ(collectFaces(geometry, 6), geometry.faces());
}

TEST(MeshGeometry, PreservesArchiveChunksAndRemovesActiveGeometry) {
  MeshGeometry geometry;
  geometry.setMaxVertices(3);
  MeshDelta first({});
  addTriangle(first, 0, true);
  addTriangle(first, 2, true);
  addTriangle(first, 4, false);
  geometry.applyDelta(first);
  geometry.clearDirty();
  const auto stored = geometry.chunks()[0].vertices.data();

  MeshDelta next({1, 3, 1});
  addTriangle(next, 6, false);
  EXPECT_EQ(geometry.applyDelta(next), 6u);
  ASSERT_EQ(geometry.chunks().size(), 3u);
  EXPECT_EQ(geometry.chunks()[0].vertices.data(), stored);
  EXPECT_FALSE(geometry.chunks()[0].dirty);
  EXPECT_EQ(collectFaces(geometry, 3), geometry.faces());
  EXPECT_FLOAT_EQ(geometry.vertices().back().pos.x(), 6.0f);

  MeshDelta empty({2, 3, 1});
  geometry.applyDelta(empty);
  EXPECT_EQ(geometry.chunks().size(), 2u);
  EXPECT_EQ(collectFaces(geometry, 3), geometry.faces());
}

TEST(MeshGeometry, PendingFacesRemapAndNormalsCrossChunkBoundaries) {
  MeshGeometry geometry;
  geometry.setMaxVertices(3);
  geometry.setNormalsEnabled(true);
  MeshDelta first({});
  first.addVertex({0, 0, 0}, {}, true);
  first.addVertex({1, 0, 0}, {}, true);
  first.addVertex({0, 1, 0}, {}, true);
  first.addVertex({5, 0, 0}, {}, true);
  first.addVertex({6, 0, 0}, {}, true);
  first.addVertex({5, 1, 0}, {}, true);
  first.addVertex({0, 0, 1}, {});
  first.addFace({0, 1, 2}, true);
  first.addFace({3, 4, 5}, true);
  first.addFace({1, 0, 6}, true);  // Pending: references an active vertex.
  geometry.applyDelta(first);
  EXPECT_TRUE(geometry.normal(0).isApprox(Eigen::Vector3f(0, 1, 1).normalized()));
  geometry.clearDirty();
  const auto stored = geometry.chunks()[0].vertices.data();

  MeshDelta next(MeshDelta::TrackingInfo::with_remap(1, 1, 0, {{0, 1}}));
  next.addVertex({5, 5, 5}, {});
  next.addVertex({0, 0, -1}, {});
  geometry.applyDelta(next);
  EXPECT_EQ(geometry.faces()[2], (traits::Face{1, 0, 7}));
  EXPECT_EQ(geometry.chunks()[0].vertices.data(), stored);
  EXPECT_EQ(collectFaces(geometry, 3), geometry.faces());
  EXPECT_TRUE(geometry.normal(0).isApprox(Eigen::Vector3f(0, -1, 1).normalized()));
  EXPECT_TRUE(geometry.chunks()[0].dirty);

  // Incremental normals must match normals computed from a fresh full mesh.
  MeshGeometry reference;
  reference.setNormalsEnabled(true);
  reference.setMesh(geometry.vertices(), geometry.faces());
  for (size_t i = 0; i < geometry.vertices().size(); ++i) {
    EXPECT_TRUE(geometry.normal(i).isApprox(reference.normal(i)));
  }
}

TEST(MeshGeometry, InvalidLeadingFaceCanBecomeValidAfterDelta) {
  MeshGeometry geometry;
  MeshDelta first({});
  first.addVertex({0, 0, 0}, {});
  first.addVertex({1, 0, 0}, {});
  first.addVertex({0, 1, 0}, {});
  first.addFace({0, 1, 9});
  first.addFace({0, 1, 2});
  geometry.applyDelta(first);

  MeshDelta next({1, 3, 2});
  addTriangle(next, 0, false);
  addTriangle(next, 2, false);
  geometry.applyDelta(next);
  EXPECT_EQ(collectFaces(geometry, 16384), geometry.faces());
}

TEST(MeshGeometry, CapsFacesEvenWhenAllVerticesAreShared) {
  MeshGeometry geometry;
  geometry.setMaxVertices(3);
  std::vector<traits::Vertex> vertices(3);
  const std::vector<traits::Face> faces(20, {0, 1, 2});
  geometry.setMesh(vertices, faces);
  EXPECT_EQ(collectFaces(geometry, 3), faces);
  EXPECT_EQ(geometry.chunks().size(), 4u);
}

TEST(MeshGeometry, LightingToggleAndInvalidFaces) {
  MeshGeometry geometry;
  std::vector<traits::Vertex> vertices(3);
  vertices[1].pos = {0, 1, 0};
  vertices[2].pos = {0, 0, 1};
  geometry.setMesh(vertices, {{0, 1, 2}, {0, 0, 0}, {0, 1, 9}});
  EXPECT_EQ(collectFaces(geometry, 16384).size(), 2u);
  EXPECT_TRUE(geometry.normal(0).isApprox(Eigen::Vector3f::UnitZ()));
  geometry.setNormalsEnabled(true);
  EXPECT_TRUE(geometry.normal(0).isApprox(Eigen::Vector3f::UnitX()));
  geometry.setNormalsEnabled(false);
  EXPECT_TRUE(geometry.normal(0).isApprox(Eigen::Vector3f::UnitZ()));
}

}  // namespace
}  // namespace kimera_pgmo
