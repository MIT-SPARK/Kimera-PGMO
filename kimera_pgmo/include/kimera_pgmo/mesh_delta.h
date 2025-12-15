#pragma once
#include <map>
#include <memory>
#include <set>
#include <vector>

#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

struct MeshOffsetInfo {
  size_t archived_vertices = 0;
  size_t prev_archived_vertices = 0;
  size_t archived_faces = 0;
};

class MeshDelta {
 public:
  using Ptr = std::unique_ptr<MeshDelta>;
  using Face = traits::Face;
  using Vertex = traits::Vertex;
  struct TrackingInfo {
    uint16_t sequence_number = 0;
    size_t prev_active_vertices = 0;
    size_t prev_active_faces = 0;
  } const info;

  explicit MeshDelta(const TrackingInfo& info);

  size_t addVertex(const traits::Pos& pos,
                   const traits::VertexTraits& traits,
                   bool archive = false);
  void addFace(const traits::Face& face, bool archive = false);

  size_t getNumVertices() const;
  size_t getNumActiveVertices() const;
  size_t getNumArchivedVertices() const;
  size_t getNumFaces() const;
  size_t getNumActiveFaces() const;
  size_t getNumArchivedFaces() const;

  const Vertex& getVertex(size_t index) const;
  const Face& getFace(size_t index) const;

  template <typename Mesh>
  static MeshDelta::Ptr fromMesh(const Mesh& mesh);

  template <typename Vertices, typename Faces>
  static MeshDelta::Ptr fromMesh(const Vertices& vertices, const Faces& faces);

  /**
   * @brief Remap vertex indices from a previous mesh (in-place)
   * @note Most useful for containers with an in-place erase method
   * @param indices Indices to remap
   * @param offsets Offset information from applying the current mesh delta
   * @param is_archived Optional output as to whether all indices are archived after
   * remap
   * @param deleted_inidces Optional output as to which entries in the indices were
   * deleted
   */
  template <template <typename T> typename ContainerT>
  void updateIndices(ContainerT<size_t>& indices,
                     const MeshOffsetInfo& offsets,
                     bool* is_archived = nullptr,
                     std::set<size_t>* deleted_indices = nullptr) const;

  /**
   * @brief Remap vertex indices from a previous mesh
   * @note Most useful for containers without an in-place erase method
   * @param indices Indices to remap
   * @param offsets Offset information from applying the current mesh delta
   * @param is_archived Optional output as to whether all indices are archived after
   * remap
   * @param deleted_indices Optional output as to which entries in the indices were
   * deleted
   */
  template <template <typename T> typename ContainerT>
  ContainerT<size_t> remapIndices(const ContainerT<size_t>&,
                                  const MeshOffsetInfo& offsets,
                                  bool* is_archived = nullptr,
                                  std::set<size_t>* deleted_indices = nullptr) const;

  template <typename Mesh>
  MeshOffsetInfo updateMesh(Mesh& mesh,
                            const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Vertices, typename Faces>
  MeshOffsetInfo updateMesh(Vertices& vertices,
                            Faces& faces,
                            const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Vertices>
  size_t updateVertices(Vertices& vertices,
                        const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Faces>
  void updateFaces(Faces& faces, size_t vertex_offset) const;

  const std::vector<Face>& face_updates() const;
  const std::vector<Face>& face_archive_updates() const;
  const std::map<size_t, size_t>& prev_to_curr() const;
  std::map<size_t, size_t>& prev_to_curr();
  const traits::VertexProperties& vertex_properties() const;

  traits::Timestamp timestamp_ns = 0;

 protected:
  friend class DeltaCompression;

  size_t num_archived_vertices_ = 0;
  std::map<size_t, size_t> prev_to_curr_;
  traits::VertexProperties vertex_properties_;

  std::vector<Vertex> vertex_updates_;
  std::vector<Face> face_updates_;
  std::vector<Face> face_archive_updates_;
};

// vertex traits
size_t pgmoNumVertices(const MeshDelta& delta);
traits::VertexProperties pgmoGetVertexProperties(const MeshDelta& delta);
traits::Pos pgmoGetVertex(const MeshDelta& delta,
                          size_t i,
                          traits::VertexTraits* traits);

// face traits
size_t pgmoNumFaces(const MeshDelta& delta);
traits::Face pgmoGetFace(const MeshDelta& delta, size_t i);

}  // namespace kimera_pgmo

#include "kimera_pgmo/impl/mesh_delta.h"  // IWYU pragma: keep
