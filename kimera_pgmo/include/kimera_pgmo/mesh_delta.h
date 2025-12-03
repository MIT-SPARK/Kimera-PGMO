#pragma once
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

class MeshDelta {
 public:
  using Ptr = std::shared_ptr<MeshDelta>;
  using Face = traits::Face;
  struct Vertex {
    traits::Pos pos;
    traits::VertexTraits traits;
  };

  MeshDelta(traits::Timestamp timestamp_ns = 0, uint16_t sequence_number = 0);

  size_t addVertex(const traits::Pos& pos,
                   const traits::VertexTraits* traits = nullptr,
                   bool archive = false);
  void addFace(const traits::Face& face, bool archive = false);

  size_t getNumVertices() const;
  size_t getNumArchivedVertices() const;
  size_t getNumFaces() const;
  size_t getNumArchivedFaces() const;

  const Vertex& getVertex(size_t index) const;
  const Face& getFace(size_t index) const;

  size_t getLocalIndex(size_t index) const;
  size_t getGlobalIndex(size_t index) const;

  template <typename Mesh>
  static MeshDelta::Ptr fromMesh(const Mesh& mesh);

  template <typename Vertices, typename Faces>
  static MeshDelta::Ptr fromMesh(const Vertices& vertices, const Faces& faces);

  template <typename Mesh>
  void updateMesh(Mesh& mesh, const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Vertices, typename Faces>
  void updateMesh(Vertices& vertices,
                  Faces& faces,
                  const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Vertices>
  void updateVertices(Vertices& vertices,
                      const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Faces>
  void updateFaces(Faces& faces) const;

  const traits::Timestamp timestamp_ns;
  const uint16_t sequence_number;

 protected:
  size_t num_archived_vertices_ = 0;

  std::vector<Vertex> vertex_updates_;
  std::vector<Face> face_updates_;
  std::vector<Face> face_archive_updates_;
  std::map<size_t, size_t> prev_to_curr_;
  std::set<size_t> deleted_indices_;
  std::set<size_t> observed_indices_;
  std::set<size_t> new_indices_;
};

}  // namespace kimera_pgmo

#include "kimera_pgmo/mesh_delta_impl.h"  // IWYU pragma: keep
