/**
 * @file   CommonFunctions.h
 * @brief  Some common functions used in library
 * @author Yun Chang
 */

#pragma once

#include <string>

#include <geometry_msgs/Pose.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/Values.h>
#include <mesh_msgs/TriangleMesh.h>
#include <pcl/PolygonMesh.h>
#include <pcl/io/ply_io.h>
#include <pcl/octree/octree_search.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_msgs/PolygonMesh.h>
#include <pose_graph_tools/PoseGraph.h>
#include <ros/console.h>
#include <voxblox_msgs/Mesh.h>

namespace kimera_pgmo {

const std::map<size_t, char> robot_id_to_prefix = {
    {0, 'a'},
    {1, 'b'},
    {2, 'c'},
    {3, 'd'},
    {4, 'e'},
    {5, 'f'},
    {6, 'g'},
    {7, 'h'},
};

const std::map<size_t, char> robot_prefix_to_id = {
    {'a', 0},
    {'b', 1},
    {'c', 2},
    {'d', 3},
    {'e', 4},
    {'f', 5},
    {'g', 6},
    {'h', 7},
};

const std::map<size_t, char> robot_id_to_vertex_prefix = {
    {0, 's'},
    {1, 't'},
    {2, 'u'},
    {3, 'v'},
    {4, 'w'},
    {5, 'x'},
    {6, 'y'},
    {7, 'z'},
};

const std::map<size_t, char> vertex_prefix_to_id = {
    {'s', 0},
    {'t', 1},
    {'u', 2},
    {'v', 3},
    {'w', 4},
    {'x', 5},
    {'y', 6},
    {'z', 7},
};

/*! \brief Get gtsam prefix for a robot id
 *  - robot_id: id of robot from pose graph messages
 */
inline char GetRobotPrefix(size_t robot_id) {
  if (!robot_id_to_prefix.count(robot_id)) {
    return '\0';
  }
  return robot_id_to_prefix.at(robot_id);
}

/*! \brief Get gtsam prefix (for mesh vertices in deformation graph) for a robot
 * id
 *  - robot_id: id of robot from pose graph messages
 */
inline char GetVertexPrefix(size_t robot_id) {
  if (!robot_id_to_vertex_prefix.count(robot_id)) {
    return '\0';
  }
  return robot_id_to_vertex_prefix.at(robot_id);
}

typedef pose_graph_tools::PoseGraph::ConstPtr GraphMsgPtr;

/*! \brief Read ply file and convert to polygon mesh type
 *  - mesh: pcl PolygonMesh pointer
 */
void ReadMeshFromPly(const std::string& filename, pcl::PolygonMeshPtr mesh);

/*! \brief Write a pcl PolygonMesh mesh to ply file
 *  - filename: name of output ply file
 *  - mesh: mesh to save
 */
void WriteMeshToPly(const std::string& filename, const pcl::PolygonMesh& mesh);

/*! \brief Convert pcl PolygonMesh to mesh_msg TriangleMesh
 *  - polygon_mesh: mesh to convert
 */
mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(
    const pcl::PolygonMesh& polygon_mesh);

/*! \brief Convert a mesh represented by vertices and polygons to a mesh_msg
 * TriangleMesh
 *  - vertices: vertices (as point cloud) of the mesh
 *  - polygons: surfaces of mesh (as pcl::Vertices polygons)
 */
mesh_msgs::TriangleMesh PolygonMeshToTriangleMeshMsg(
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const std::vector<pcl::Vertices>& polygons);

/*! \brief Convert a mesh_msg TriangleMesh to PolygonMesh
 *  - mesh_msg: TriangleMesh mesh to be converted
 *  - outputs mesh as PolygonMesh type
 */
pcl::PolygonMesh TriangleMeshMsgToPolygonMesh(
    const mesh_msgs::TriangleMesh& mesh_msg);

/*! \brief Converts a ros pose type to gtsam Pose3
 *  - transform: ros geometry_msgs pose type
 *  - outputs pose as gtsam Pose3
 */
gtsam::Pose3 RosToGtsam(const geometry_msgs::Pose& transform);

/*! \brief Converts a gtsam pose type to ros geometry message
 *  - pose: gtsam pose3
 *  - outputs pose as geometry_msg/pose
 */
geometry_msgs::Pose GtsamToRos(const gtsam::Pose3& pose);

/*! \brief Converts a pcl point to gtsam point3
 *  - point: pcl point
 *  - outputs gtsam point3 position
 */
template <class point_type>
inline gtsam::Point3 PclToGtsam(const point_type& point) {
  return gtsam::Point3(static_cast<double>(point.x),
                       static_cast<double>(point.y),
                       static_cast<double>(point.z));
}

/*! \brief Converts a gtsam point3 to a pcl point
 *  - point: gtsam point3
 *  - outputs pcl point
 */
template <class point_type>
inline point_type GtsamToPcl(const gtsam::Point3& point) {
  return point_type(static_cast<double>(point.x()),
                    static_cast<double>(point.y()),
                    static_cast<double>(point.z()));
}

/*! \brief Combine two meshes into one
 *  - mesh1: partial mesh represented as pcl PolygonMesh
 *  - mesh2: partial mesh represented as pcl PolygonMesh
 *  - check_duplicate_vertices: boolean to check if vertices are duplicated or
 * not
 *  - outputs combined mesh as pcl PolygonMesh
 */
pcl::PolygonMesh CombineMeshes(const pcl::PolygonMesh& mesh1,
                               const pcl::PolygonMesh& mesh2,
                               bool check_duplicate_vertices = true);

/*! \brief Combine two meshes into one
 *  - mesh1: artial mesh represented as pcl PolygonMesh
 *  - mesh2: partial mesh represented as pcl PolygonMesh
 *  - indices_to_check: check mesh2 vertices against vertices at these indices
 * in mesh1 for duplication
 *  - vertex_indices: indices of the mesh2 vertices in the combined mesh
 *  - outputs combined mesh as pcl PolygonMesh
 */
pcl::PolygonMesh CombineMeshes(const pcl::PolygonMesh& mesh1,
                               const pcl::PolygonMesh& mesh2,
                               const std::vector<size_t>& indices_to_check,
                               std::vector<size_t>* vertex_indices);

/*! \brief Check if two polygons are equal to each other
 *  - p1: polygon represented as pcl Vertices
 *  - p2: polygon represented as pcl Vertices
 */
bool PolygonsEqual(const pcl::Vertices& p1, const pcl::Vertices& p2);

/*! \brief Convert gtsam factor graph and estimated values to ros graph msg for
 * publication
 *  - factors: gtsam NonlinearFactorGraph storing all added factors
 *  - values: gtsam Values storing estimated values
 *  - timestamps: map from robot id to timestamps of the poses in the
 * trajectories
 */
GraphMsgPtr GtsamGraphToRos(
    const gtsam::NonlinearFactorGraph& factors,
    const gtsam::Values& values,
    const std::map<size_t, std::vector<ros::Time>>& timestamps);

/*! \brief Check if a surface exist based on previous tracked adjacent surfaces
 *  - new_surface: new surface to be inserted
 *  - adjacent_surfaces: vertex to adjacent surfaces (should correspond exactly
 * to order of vertices)
 *  returns true if surface exists
 */
bool SurfaceExists(
    const pcl::Vertices& new_surface,
    const std::map<size_t, std::vector<size_t>>& adjacent_surfaces,
    const std::vector<pcl::Vertices>& surfaces);

/*! \brief Check if a point is within the bounding box of an octree structure
 *  - octree: pcl octree type
 *  - p: point to query
 */
template <class point_type>
bool InOctreeBoundingBox(
    const pcl::octree::OctreePointCloudSearch<point_type>& octree,
    const point_type& p) {
  double min_x, min_y, min_z, max_x, max_y, max_z;
  octree.getBoundingBox(min_x, min_y, min_z, max_x, max_y, max_z);
  return (p.x >= min_x && p.x <= max_x) && (p.y >= min_y && p.y <= max_y) &&
         (p.z >= min_z && p.z <= max_z);
}

/*! \brief Deform a points (i.e. the vertices of a mesh) based on the
 * controls points via deformation
 * - original_points: set of points to deform
 * - prefix: a char to distinguish the type of control points
 * - control_points: original positions of the control points. In the case of
 * mesh vertices, these are the original positions of the simplified mesh.
 * - values: key-value pairs. Where each key should be gtsam::Symbol(prefix,
 * idx-in-control-points) from the previous two arguments.
 * - k: how many nearby nodes to use to adjust new position of vertices
 */
template <class point_type>
pcl::PointCloud<point_type> deformPoints(
    const pcl::PointCloud<point_type>& original_points,
    const char& prefix,
    const std::vector<gtsam::Point3>& control_points,
    const gtsam::Values& values,
    size_t k = 4) {
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;
  // Check if there are points to deform
  if (original_points.size() == 0) {
    return original_points;
  }

  // Cannot deform if no nodes in the deformation graph
  if (control_points.size() == 0) {
    ROS_WARN("No control points. No deformation. ");
    return original_points;
  }

  pcl::PointCloud<point_type> new_points;

  // Build Octree
  Octree::Ptr search_octree(new Octree(1.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(
      new pcl::PointCloud<pcl::PointXYZ>);
  search_octree->setInputCloud(search_cloud);
  for (size_t j = 0; j < control_points.size(); j++) {
    const gtsam::Point3& position = control_points[j];
    search_cloud->push_back(
        pcl::PointXYZ(position.x(), position.y(), position.z()));
    if (!values.exists(gtsam::Symbol(prefix, j))) continue;
    search_octree->addPointFromCloud(search_cloud->points.size() - 1, nullptr);
  }

  if (search_octree->getLeafCount() < k) {
    ROS_WARN("Not enough valid control points to deform points. ");
    return original_points;
  }

  for (size_t ii = 0; ii < original_points.size(); ii++) {
    const point_type& p = original_points.points[ii];
    // search for k + 1 nearest nodes
    std::vector<std::pair<size_t, double>> nearest_nodes;
    pcl::PointXYZ p_xyz(p.x, p.y, p.z);
    gtsam::Point3 vi(p.x, p.y, p.z);
    // Query octree
    std::vector<int> nearest_nodes_index;
    std::vector<float> nearest_nodes_sq_dist;
    search_octree->nearestKSearch(
        p_xyz, k + 1, nearest_nodes_index, nearest_nodes_sq_dist);

    // Calculate new point location from k points
    gtsam::Point3 new_point(0, 0, 0);
    double d_max =
        std::sqrt(nearest_nodes_sq_dist[nearest_nodes_index.size() - 1]);
    double weight_sum = 0;
    for (size_t j = 0; j < nearest_nodes_index.size() - 1; j++) {
      const pcl::PointXYZ& p_g =
          search_cloud->points.at(nearest_nodes_index[j]);
      gtsam::Point3 gj(p_g.x, p_g.y, p_g.z);
      double weight = (1 - std::sqrt(nearest_nodes_sq_dist[j]) / d_max);
      if (weight_sum == 0 && weight == 0) weight = 1;
      weight_sum = weight_sum + weight;
      gtsam::Pose3 node_transform = values.at<gtsam::Pose3>(
          gtsam::Symbol(prefix, nearest_nodes_index[j]));
      gtsam::Point3 add = node_transform.rotation().rotate(vi - gj) +
                          node_transform.translation();
      new_point = new_point + weight * add;
    }
    // Add back to new_vertices
    point_type new_pcl_point = p;
    new_pcl_point.x = new_point.x() / weight_sum;
    new_pcl_point.y = new_point.y() / weight_sum;
    new_pcl_point.z = new_point.z() / weight_sum;
    new_points.points.push_back(new_pcl_point);
  }

  return new_points;
}

}  // namespace kimera_pgmo
