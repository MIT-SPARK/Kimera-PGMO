#pragma once
#include <kimera_pgmo/mesh_delta.h>

#include <optional>
#include <set>
#include <vector>

namespace kimera_pgmo {

//! CPU mesh and render partitioning, independent of Ogre.
class MeshGeometry {
 public:
  struct Chunk {
    size_t first_face = 0;
    std::vector<size_t> vertices;
    std::vector<traits::Face> faces;
    bool dirty = true;
  };

  void setMesh(std::vector<traits::Vertex> vertices, std::vector<traits::Face> faces);
  //! Returns the first changed vertex in the canonical mesh.
  //! Non-sequential deltas replace active and pending geometry, retaining archives.
  size_t applyDelta(const MeshDelta& delta);
  void setMaxVertices(size_t max_vertices);
  void setNormalsEnabled(bool enabled);
  void markAllDirty();
  void clearDirty();

  const auto& vertices() const { return vertices_; }
  const auto& faces() const { return faces_; }
  const auto& chunks() const { return chunks_; }
  Eigen::Vector3f normal(size_t vertex) const;

 private:
  bool needsRecovery(uint16_t sequence_number) const;
  void validateDelta(const MeshDelta& delta) const;
  void rebuildChunks(size_t keep);
  void appendChunks(size_t first, size_t end);
  void updateNormals(size_t first_face, float sign, std::set<size_t>& changed);
  void dirtyNormals(const std::set<size_t>& changed);

  size_t max_vertices_ = 16384;
  size_t archived_chunks_ = 0;
  bool normals_enabled_ = false;
  MeshOffsetInfo offsets_;
  std::optional<uint16_t> sequence_number_;
  std::vector<traits::Vertex> vertices_;
  std::vector<traits::Face> faces_;
  std::vector<Chunk> chunks_;
  std::vector<std::vector<size_t>> vertex_chunks_;
  std::vector<Eigen::Vector3f> normals_;
};

}  // namespace kimera_pgmo
