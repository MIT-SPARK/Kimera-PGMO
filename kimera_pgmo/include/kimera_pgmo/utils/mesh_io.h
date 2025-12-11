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
#include "kimera_pgmo/mesh_types.h"
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
  std::vector<Timestamp> first_seen_stamps;
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
  const auto props = traits::get_vertex_properties(vertices);
  const auto num_vertices = traits::num_vertices(vertices);

  IOData data;
  for (size_t i = 0; i < num_vertices; ++i) {
    traits::VertexTraits traits;
    const auto pos = traits::get_vertex(vertices, i, &traits);

    data.x.push_back(pos.x());
    data.y.push_back(pos.y());
    data.z.push_back(pos.z());

    if (props.has_color) {
      data.r.push_back(traits.color[0]);
      data.g.push_back(traits.color[1]);
      data.b.push_back(traits.color[2]);
      data.a.push_back(traits.color[3]);
    }

    if (props.has_stamp) {
      data.stamps.push_back(traits.stamp);
    }

    if (props.has_label) {
      data.labels.push_back(traits.label);
    }

    if (props.has_first_seen_stamp) {
      data.first_seen_stamps.push_back(traits.first_seen_stamp);
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
  traits::VertexProperties props;
  props.has_color = data.r.size() == num_vertices;
  props.has_stamp = data.stamps.size() == num_vertices;
  props.has_label = data.labels.size() == num_vertices;
  props.has_first_seen_stamp = data.first_seen_stamps.size() == num_vertices;

  traits::resize_vertices(vertices, num_vertices);
  for (size_t i = 0; i < num_vertices; i++) {
    const traits::Pos pos(data.x[i], data.y[i], data.z[i]);

    traits::VertexTraits traits;
    traits.properties = props;
    if (traits.properties.has_color) {
      const auto alpha = i < data.a.size() ? data.a[i] : static_cast<uint8_t>(255);
      traits.color = traits::Color{data.r[i], data.g[i], data.b[i], alpha};
    }

    if (traits.properties.has_stamp) {
      traits.stamp = data.stamps[i];
    }

    if (traits.properties.has_label) {
      traits.label = data.labels[i];
    }

    if (traits.properties.has_first_seen_stamp) {
      traits.first_seen_stamp = data.first_seen_stamps[i];
    }

    traits::set_vertex(vertices, i, pos, &traits);
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
