/**
 * @file   mesh_io.h
 * @brief  Functions to read and write meshes from files
 * @author Yun Chang
 */

#pragma once

#include <pcl/PolygonMesh.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <string>

#include "kimera_pgmo/mesh_traits.h"
#include "kimera_pgmo/utils/common_structs.h"

namespace kimera_pgmo {

struct IOData {
  using Ptr = std::shared_ptr<IOData>;
  std::vector<float> x;
  std::vector<float> y;
  std::vector<float> z;
  std::vector<uint8_t> r;
  std::vector<uint8_t> g;
  std::vector<uint8_t> b;
  std::vector<uint8_t> a;
  std::vector<Timestamp> stamps;
  std::vector<uint32_t> labels;
  std::vector<std::vector<uint32_t>> faces;

  static IOData::Ptr load(const std::string& filename);

  void save(const std::string& filename) const;
};

/*! \brief Read ply file and convert to polygon mesh type
 *  - mesh: pcl PolygonMesh pointer
 */
void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh);

/*! \brief Read ply file and convert to polygon mesh type
 *  - mesh: pcl PolygonMesh pointer
 *  - vertex_stamps: timestamps associated with mesh vertices
 */
void ReadMeshWithStampsFromPly(const std::string& filename,
                               pcl::PolygonMesh& mesh,
                               std::vector<Timestamp>* vertex_stamps = nullptr);

/*! \brief Read ply file and convert to polygon mesh type
 *  - mesh: pcl PolygonMesh pointer
 *  - vertex_stamps: timestamps associated with mesh vertices
 */
void ReadMeshWithStampsFromPly(const std::string& filename,
                               pcl::PolygonMeshPtr mesh,
                               std::vector<Timestamp>* vertex_stamps = nullptr);

/*! \brief Write a pcl PolygonMesh mesh to ply file
 *  - filename: name of output ply file
 *  - mesh: mesh to save
 */
void WriteMeshToPly(const std::string& filename, const pcl::PolygonMesh& mesh);

/*! \brief Write a pcl PolygonMesh mesh to ply file along with vertex timestamps
 *  - filename: name of output ply file
 *  - mesh: mesh to save
 *  - vertex_stamps: vector of vertex timestamps
 */
void WriteMeshWithStampsToPly(const std::string& filename,
                              const pcl::PolygonMesh& mesh,
                              const std::vector<Timestamp>& vertex_stamps);

template <typename Vertices, typename Faces>
void WriteMesh(const std::string& filename,
               const Vertices& vertices,
               const Faces& faces) {
  const auto num_vertices = traits::num_vertices(vertices);
  IOData data;
  for (size_t i = 0; i < num_vertices; ++i) {
    std::optional<traits::Color> color;
    std::optional<uint8_t> alpha;
    std::optional<uint64_t> stamp;
    std::optional<uint32_t> label;
    const auto pos = traits::get_vertex(vertices, i, &color, &alpha, &stamp, &label);
    data.x.push_back(pos.x());
    data.y.push_back(pos.y());
    data.z.push_back(pos.z());

    if (color) {
      const auto& c = *color;
      data.r.push_back(c[0]);
      data.g.push_back(c[1]);
      data.b.push_back(c[2]);
    }

    if (alpha) {
      data.a.push_back(*alpha);
    }

    if (stamp) {
      data.stamps.push_back(*stamp);
    }

    if (label) {
      data.labels.push_back(*label);
    }
  }

  const auto num_faces = traits::num_faces(faces);
  for (size_t i = 0; i < num_faces; ++i) {
    const auto tri = traits::get_face(faces, i);
    auto& new_face = data.faces.emplace_back();
    new_face.push_back(tri[0]);
    new_face.push_back(tri[1]);
    new_face.push_back(tri[2]);
  }

  data.save(filename);
}

template <typename Vertices, typename Faces>
void ReadMesh(const std::string& filename, Vertices& vertices, Faces& faces) {
  const auto data_ptr = IOData::load(filename);
  if (!data_ptr) {
    return;
  }

  const auto& data = *data_ptr;
  const auto num_vertices = data.x.size();
  const bool has_colors = data.r.size() == num_vertices;
  const bool has_stamps = data.stamps.size() == num_vertices;
  const bool has_labels = data.labels.size() == num_vertices;

  traits::resize_vertices(vertices, num_vertices);
  for (size_t i = 0; i < num_vertices; i++) {
    const traits::Pos pos(data.x[i], data.y[i], data.z[i]);

    std::optional<traits::Color> color;
    std::optional<uint8_t> alpha;
    if (has_colors) {
      color = traits::Color{{data.r[i], data.g[i], data.b[i]}};
      if (i < data.a.size()) {
        alpha = data.a[i];
      }
    }

    std::optional<uint64_t> timestamp;
    if (has_stamps) {
      timestamp = data.stamps[i];
    }

    std::optional<uint32_t> label;
    if (has_labels) {
      label = data.labels[i];
    }

    traits::set_vertex(vertices, i, pos, color, alpha, timestamp, label);
  }

  traits::resize_faces(faces, data.faces.size());
  for (size_t i = 0; i < data.faces.size(); ++i) {
    const auto& face_vec = data.faces[i];
    traits::Face face{face_vec[0], face_vec[1], face_vec[2]};
    traits::set_face(faces, i, face);
  }
}

template <typename Mesh>
void WriteMesh(const std::string& filename, const Mesh& mesh) {
  // dispatch types that have both vertices and face adl api
  WriteMesh(filename, mesh, mesh);
}

template <typename Mesh>
void ReadMesh(const std::string& filename, Mesh& mesh) {
  // dispatch types that have both vertices and face adl api
  ReadMesh(filename, mesh, mesh);
}

}  // namespace kimera_pgmo
