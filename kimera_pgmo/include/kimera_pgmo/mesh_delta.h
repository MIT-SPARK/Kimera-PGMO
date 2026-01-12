#pragma once
#include <map>
#include <memory>
#include <vector>

#include <Eigen/Geometry>

#include "kimera_pgmo/mesh_offset_info.h"
#include "kimera_pgmo/mesh_types.h"

namespace kimera_pgmo {

class MeshDelta {
 public:
  using Ptr = std::unique_ptr<MeshDelta>;
  using Face = traits::Face;
  using Vertex = traits::Vertex;

  //! Tracking information for the current mesh delta
  struct TrackingInfo {
    //! Index of delta in sequence of deltas (used by compression algorithm)
    uint16_t sequence_number = 0;
    //! Number of active vertices in the previous delta (used for archival)
    size_t prev_active_vertices = 0;
    //! Number of active faces in the previous delta (used for archival)
    size_t prev_active_faces = 0;
    //! Mapping between previous delta active indices and current delta
    std::shared_ptr<std::map<size_t, size_t>> prev_to_curr = nullptr;

    static TrackingInfo with_remap(uint16_t sequence_number,
                                   size_t prev_active_vertices,
                                   size_t prev_active_faces,
                                   const std::map<size_t, size_t>& remap = {});
  } const info;

  //! Construct a mesh delta from tracking information
  explicit MeshDelta(const TrackingInfo& info);

  /**
   * @brief Construct a delta containing active vertices and faces from a full mesh
   * @tparam MeshT Mesh type that supports PGMO traits
   * @param[in] mesh Mesh to encode into the delta
   * @param[in] info Optional tracking info for the mesh
   * @return Encoded mesh delta
   */
  template <typename MeshT>
  static MeshDelta::Ptr fromMesh(const MeshT& mesh, const TrackingInfo* info = nullptr);

  /**
   * @brief Construct a delta containing active vertices and faces from a full mesh
   * @tparam VerticesT Vertex container that supports PGMO traits
   * @tparam FacesT Face container that supports PGMO traits
   * @param[in] vertices Mesh vertices to encode into the delta
   * @param[in] faces Mesh faces to encode into the delta
   * @param[in] info Optional tracking info for the mesh
   * @return Encoded mesh delta
   */
  template <typename VerticesT, typename FacesT>
  static MeshDelta::Ptr fromMesh(const VerticesT& vertices,
                                 const FacesT& faces,
                                 const TrackingInfo* info = nullptr);

  /**
   * @brief Add a new vertex to the mesh delta, optionally archiving it
   * @note All archived vertices must be added before the active vertices
   * @param[in] pos Vertex position
   * @param[in] traits Vertex properties to add
   * @param[in] archive Whether or not the vertex is archived
   * @return Local index of the added vertex
   */
  size_t addVertex(const traits::Pos& pos,
                   const traits::VertexTraits& traits,
                   bool archive = false);

  /**
   * @brief Add a new face to the mesh delta, optionally archiving it
   * @param[in] face New face to add
   * @param[in] archive Whether or not the face is archived
   */
  void addFace(const traits::Face& face, bool archive = false);

  //! Get a vertex in the delta by local index
  const Vertex& getVertex(size_t index) const;
  //! Get a face in the delta by local index
  const Face& getFace(size_t index) const;

  //! Get the total number of vertices in the delta
  size_t getNumVertices() const;
  //! Get the number of unarchived vertices in the delta
  size_t getNumActiveVertices() const;
  //! Get the number of archived vertices in the delta
  size_t getNumArchivedVertices() const;

  //! Get the total number of faces in the delta
  size_t getNumFaces() const;
  //! Get the number of unarchived faces in the delta
  size_t getNumActiveFaces() const;
  //! Get the number of archived faces in the delta
  size_t getNumArchivedFaces() const;

  template <typename Mesh>
  void updateMesh(Mesh& mesh,
                  MeshOffsetInfo& offsets,
                  const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Vertices, typename Faces>
  void updateMesh(Vertices& vertices,
                  Faces& faces,
                  MeshOffsetInfo& offsets,
                  const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Vertices>
  size_t updateVertices(Vertices& vertices,
                        const Eigen::Isometry3f* transform = nullptr) const;

  template <typename Faces>
  size_t updateFaces(Faces& faces,
                     const MeshOffsetInfo& prev_offests,
                     size_t vertex_offset) const;

  //! Get the valid vertex properties that the delta has
  const traits::VertexProperties& vertex_properties() const;
  //! Get the active faces of the delta
  const std::vector<Face>& faces() const;
  //! Get the archived faces of the delta
  const std::vector<Face>& archived_faces() const;

  //! Last updated timestamp of the mesh delta
  traits::Timestamp timestamp_ns = 0;

 protected:
  size_t num_archived_vertices_ = 0;
  traits::VertexProperties vertex_properties_;

  std::vector<Vertex> vertex_updates_;
  std::vector<Face> face_updates_;
  std::vector<Face> face_archive_updates_;
};

//! Get the number of vertices in the mesh delta
size_t pgmoNumVertices(const MeshDelta& delta);

//! Get the valid vertex properties in the mesh delta
traits::VertexProperties pgmoGetVertexProperties(const MeshDelta& delta);

//! Get a specific vertex from the mesh delta
traits::Pos pgmoGetVertex(const MeshDelta& delta,
                          size_t i,
                          traits::VertexTraits* traits);

//! Get the number of faces in the mesh delta
size_t pgmoNumFaces(const MeshDelta& delta);

//! Get a specific face from the mesh delta
traits::Face pgmoGetFace(const MeshDelta& delta, size_t i);

}  // namespace kimera_pgmo

#include "kimera_pgmo/impl/mesh_delta.h"  // IWYU pragma: keep
