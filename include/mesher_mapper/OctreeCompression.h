/**
 * @file   OctreeCompression.h
 * @brief  Simplify and reconstruct meshes Peng and Kuo 2005
 * @author Yun Chang
 */
#include <pcl/octree/octree.h>
#include <pcl/octree/octree_impl.h>

#include "mesher_mapper/MeshCompression.h"
#include "mesher_mapper/Polygon.h"

namespace mesher_mapper {

// typedef std::vector<Vertex> OctreeBlock;
// typedef std::shared_ptr<std::vector<Vertex>> OctreeBlockPtr;
typedef pcl::octree::OctreePointCloud<pcl::PointXYZ> Octree;

class OctreeCompression : public MeshCompression {
 public:
  OctreeCompression();
  ~OctreeCompression();

  bool setInputMesh(pcl::PolygonMeshPtr input_mesh);
  bool process();

 private:
  bool reset(const pcl::PolygonMesh& mesh);

  // void splitIntoEight(
  //     const OctreeBlockPtr& parent,
  //     std::shared_ptr<std::map<Vertex, size_t>> vertex_to_block_id) const;

  // std::map<OctreeBlockPtr, std::vector<OctreeBlockPtr>> octree_;
  // std::map<OctreeBlockPtr, size_t> level_;
  std::map<size_t, Graph> level_graph_;

  Octree::Ptr octree_; 

  size_t level_of_detail_;
};
}  // namespace mesher_mapper