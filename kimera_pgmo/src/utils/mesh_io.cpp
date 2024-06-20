/**
 * @file   mesh_io.cpp
 * @brief  Functions to read and write meshes from files
 * @author Yun Chang
 */
#include "kimera_pgmo/utils/mesh_io.h"

#include <pcl/conversions.h>

#include <chrono>

#include "kimera_pgmo/pcl_mesh_traits.h"
#include "kimera_pgmo/utils/happly/happly.h"

namespace kimera_pgmo {

void IOData::save(const std::string& filename) const {
  std::filebuf fb_ascii;
  fb_ascii.open(filename, std::ios::out);
  std::ostream outstream_ascii(&fb_ascii);
  if (outstream_ascii.fail()) {
    throw std::runtime_error("failed to open '" + filename + "'");
  }

  happly::PLYData output_file;
  output_file.addElement("vertex", x.size());
  output_file.getElement("vertex").addProperty<float>("x", x);
  output_file.getElement("vertex").addProperty<float>("y", y);
  output_file.getElement("vertex").addProperty<float>("z", z);
  if (r.size() == x.size() && g.size() == x.size() && b.size() == x.size()) {
    output_file.getElement("vertex").addProperty<uint8_t>("red", r);
    output_file.getElement("vertex").addProperty<uint8_t>("green", g);
    output_file.getElement("vertex").addProperty<uint8_t>("blue", b);
  }

  if (a.size() == x.size()) {
    output_file.getElement("vertex").addProperty<uint8_t>("alpha", a);
  }

  if (!stamps.empty()) {
    // Write vertex stamps to ply
    std::vector<uint32_t> stamps_sec;
    std::vector<uint32_t> stamps_nsec;
    for (const auto vertex_ns : stamps) {
      auto t = std::chrono::nanoseconds(vertex_ns);
      auto t_s = std::chrono::duration_cast<std::chrono::seconds>(t);
      std::chrono::nanoseconds t_ns = t - t_s;
      stamps_sec.push_back(t_s.count());
      stamps_nsec.push_back(t_ns.count());
    }
    output_file.getElement("vertex").addProperty<uint32_t>("secs", stamps_sec);
    output_file.getElement("vertex").addProperty<uint32_t>("nsecs", stamps_nsec);
  }

  if (!labels.empty()) {
    output_file.getElement("vertex").addProperty<uint32_t>("label", labels);
  }

  output_file.addElement("face", faces.size());
  output_file.getElement("face").addListProperty("vertex_indices", faces);
  output_file.write(filename, happly::DataFormat::ASCII);
}

IOData::Ptr IOData::load(const std::string& filename) {
  auto to_return = std::make_shared<IOData>();

  // Get data from the object
  happly::PLYData ply_in(filename);
  const auto elements = ply_in.getElementNames();
  to_return->x = ply_in.getElement("vertex").getProperty<float>("x");
  to_return->y = ply_in.getElement("vertex").getProperty<float>("y");
  to_return->z = ply_in.getElement("vertex").getProperty<float>("z");

  try {
    to_return->r = ply_in.getElement("vertex").getProperty<uint8_t>("red");
    to_return->g = ply_in.getElement("vertex").getProperty<uint8_t>("green");
    to_return->b = ply_in.getElement("vertex").getProperty<uint8_t>("blue");
    to_return->a = ply_in.getElement("vertex").getProperty<uint8_t>("alpha");
  } catch (...) {
  }

  try {
    const auto sec = ply_in.getElement("vertex").getProperty<uint32_t>("secs");
    const auto nsec = ply_in.getElement("vertex").getProperty<uint32_t>("nsecs");
    assert(sec.size() == nsec.size());

    for (size_t i = 0; i < sec.size(); i++) {
      to_return->stamps.push_back(stampFromSec(sec.at(i)) + nsec.at(i));
    }
  } catch (...) {
  }

  try {
    to_return->labels = ply_in.getElement("vertex").getProperty<uint32_t>("label");
  } catch (...) {
  }

  to_return->faces =
      ply_in.getElement("face").getListPropertyAnySign<uint32_t>("vertex_indices");

  return to_return;
}

void ReadMeshWithStampsFromPly(const std::string& filename,
                               pcl::PolygonMeshPtr mesh,
                               std::vector<Timestamp>* vertex_stamps) {
  if (!mesh) {
    return;
  }

  ReadMeshWithStampsFromPly(filename, *mesh, vertex_stamps);
}

void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh) {
  ReadMeshWithStampsFromPly(filename, mesh);
}

void WriteMeshToPly(const std::string& filename, const pcl::PolygonMesh& mesh) {
  std::vector<Timestamp> unused;
  WriteMeshWithStampsToPly(filename, mesh, unused);
}

void WriteMeshWithStampsToPly(const std::string& filename,
                              const pcl::PolygonMesh& mesh,
                              const std::vector<Timestamp>& stamps) {
  // Get point cloud
  pcl::PointCloud<pcl::PointXYZRGBA> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);

  ConstStampedCloud<pcl::PointXYZRGBA> to_write{cloud, stamps};
  WriteMesh(filename, to_write, mesh.polygons);
}

void ReadMeshWithStampsFromPly(const std::string& filename,
                               pcl::PolygonMesh& mesh,
                               std::vector<Timestamp>* stamps) {
  pcl::PointCloud<pcl::PointXYZRGBA> vertices;
  if (stamps) {
    StampedCloud<pcl::PointXYZRGBA> cloud{vertices, *stamps};
    ReadMesh(filename, cloud, mesh.polygons);
  } else {
    ReadMesh(filename, vertices, mesh.polygons);
  }

  pcl::toPCLPointCloud2(vertices, mesh.cloud);
}

}  // namespace kimera_pgmo
