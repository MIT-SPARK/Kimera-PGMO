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
#include <kimera_pgmo/KimeraPgmoMesh.h>
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
#include "kimera_pgmo/utils/CommonStructs.h"

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

const std::map<char, size_t> robot_prefix_to_id = {
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

const std::map<char, size_t> vertex_prefix_to_id = {
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

/*! \brief Read ply file and convert to polygon mesh type
 *  - mesh: pcl PolygonMesh pointer
 *  - vertex_stamps: timestamps associated with mesh vertices
 */
void ReadMeshWithStampsFromPly(const std::string& filename,
                               pcl::PolygonMesh& mesh,
                               std::vector<ros::Time>* vertex_stamps = nullptr);

/*! \brief Read ply file and convert to polygon mesh type
 *  - mesh: pcl PolygonMesh pointer
 *  - vertex_stamps: timestamps associated with mesh vertices
 */
void ReadMeshWithStampsFromPly(const std::string& filename,
                               pcl::PolygonMeshPtr mesh,
                               std::vector<ros::Time>* vertex_stamps = nullptr);

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
                              const std::vector<ros::Time>& vertex_stamps);

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

/*! \brief Convert a mesh represented by vertices and polygons to a pgmo mesh
 * msg
 *  - id: robot id
 *  - polygon_mesh: mesh to convert
 *  - vertex_timestamps: timestamps of each of the vertices
 *  - frame_id: frame id for msg header
 */
KimeraPgmoMesh PolygonMeshToPgmoMeshMsg(const size_t& id,
                                        const pcl::PolygonMesh& polygon_mesh,
                                        const std::vector<ros::Time>& vertex_timestamps,
                                        const std::string& frame_id);

/*! \brief Convert a mesh represented by vertices and polygons to a pgmo mesh
 * msg
 *  - id: robot id
 *  - vertices: vertices (as point cloud) of the mesh
 *  - polygons: surfaces of mesh (as pcl::Vertices polygons)
 *  - vertex_timestamps: timestamps of each of the vertices
 *  - frame_id: frame id for msg header
 */
KimeraPgmoMesh PolygonMeshToPgmoMeshMsg(
    const size_t& id,
    const pcl::PointCloud<pcl::PointXYZRGBA>& vertices,
    const std::vector<pcl::Vertices>& polygons,
    const std::vector<ros::Time>& vertex_timestamps,
    const std::string& frame_id,
    const IndexMapping& vertex_index_mappings = std::unordered_map<size_t, size_t>());

/*! \brief Convert a mesh_msg TriangleMesh to PolygonMesh
 *  - mesh_msg: TriangleMesh mesh to be converted
 *  - outputs mesh as PolygonMesh type
 */
pcl::PolygonMesh TriangleMeshMsgToPolygonMesh(const mesh_msgs::TriangleMesh& mesh_msg);

/*! \brief Convert a mesh_msg KimeraPgmoMesh to PolygonMesh
 *  - mesh_msg: TriangleMesh mesh to be converted
 *  - vertex_stamps: pointer to a vector of vertex timestamps
 *  - vertex_graph_indices: corresponding mesh graph vertex of vertices
 *  - outputs mesh as PolygonMesh type
 */
pcl::PolygonMesh PgmoMeshMsgToPolygonMesh(const KimeraPgmoMesh& mesh_msg,
                                          std::vector<ros::Time>* vertex_stamps,
                                          std::vector<int>* vertex_graph_indices);

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

/*! \brief Append vertices and faces of one mesh to another
 *  - mesh_vertices: vertices to append to
 *  - mesh_faces: mesh faces to append to
 *  - vertices_to_add: vertices to append
 *  - faces_to_add: faces to add
 */
void AppendMesh(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr mesh_vertices,
                std::shared_ptr<std::vector<pcl::Vertices>> mesh_faces,
                const pcl::PointCloud<pcl::PointXYZRGBA>& vertices_to_add,
                const std::vector<pcl::Vertices>& faces_to_add);

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
 *  - gnc_weights: weights obtained from gnc optimizer for inlier-outlier
 * sorting
 */
GraphMsgPtr GtsamGraphToRos(const gtsam::NonlinearFactorGraph& factors,
                            const gtsam::Values& values,
                            const std::map<size_t, std::vector<ros::Time>>& timestamps,
                            const gtsam::Vector& gnc_weights = gtsam::Vector());

/*! \brief Check if a surface exist based on previous tracked adjacent surfaces
 *  - new_surface: new surface to be inserted
 *  - adjacent_surfaces: vertex to adjacent surfaces (should correspond exactly
 * to order of vertices)
 *  returns true if surface exists
 */
bool SurfaceExists(const pcl::Vertices& new_surface,
                   const std::map<size_t, std::vector<size_t>>& adjacent_surfaces,
                   const std::vector<pcl::Vertices>& surfaces);

/*! \brief Check if a point is within the bounding box of an octree structure
 *  - octree: pcl octree type
 *  - p: point to query
 */
template <class point_type>
bool InOctreeBoundingBox(const pcl::octree::OctreePointCloudSearch<point_type>& octree,
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
void deformPoints(pcl::PointCloud<point_type>& new_points,
                  const pcl::PointCloud<point_type>& points,
                  const char& prefix,
                  const std::vector<gtsam::Point3>& control_points,
                  const gtsam::Values& values,
                  size_t k = 4,
                  const std::vector<size_t>* indices = nullptr) {
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;
  // Check if there are points to deform
  const size_t num_points = indices ? indices->size() : points.size();
  if (!num_points) {
    return;
  }

  // Cannot deform if no nodes in the deformation graph
  if (control_points.size() == 0) {
    ROS_WARN("No control points. No deformation. ");
    return;
  }

  // Build Octree
  Octree::Ptr search_octree(new Octree(1.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  search_octree->setInputCloud(search_cloud);
  for (size_t j = 0; j < control_points.size(); j++) {
    const gtsam::Point3& position = control_points[j];
    search_cloud->push_back(pcl::PointXYZ(position.x(), position.y(), position.z()));
    if (!values.exists(gtsam::Symbol(prefix, j))) continue;
    search_octree->addPointFromCloud(search_cloud->points.size() - 1, nullptr);
  }

  if (search_octree->getLeafCount() < k) {
    ROS_WARN("Not enough valid control points to deform points. ");
    return;
  }

  for (size_t point_index = 0; point_index < num_points; ++point_index) {
    const size_t ii = indices ? indices->at(point_index) : point_index;
    const point_type& p = points[ii];
    auto& p_new = new_points[ii];
    // search for k + 1 nearest nodes
    std::vector<std::pair<size_t, double>> nearest_nodes;
    const pcl::PointXYZ p_xyz(p.x, p.y, p.z);
    const gtsam::Point3 vi(p.x, p.y, p.z);
    // Query octree
    std::vector<int> nearest_nodes_index;
    std::vector<float> nearest_nodes_sq_dist;
    search_octree->nearestKSearch(
        p_xyz, k + 1, nearest_nodes_index, nearest_nodes_sq_dist);

    // Calculate new point location from k points
    gtsam::Point3 new_point(0, 0, 0);
    double d_max = std::sqrt(nearest_nodes_sq_dist[nearest_nodes_index.size() - 1]);
    if (d_max == 0) {
      continue;
    }
    bool use_const_weight = false;
    if (std::sqrt(nearest_nodes_sq_dist[0]) == d_max) {
      use_const_weight = true;
    }
    double weight_sum = 0;
    for (size_t j = 0; j < nearest_nodes_index.size() - 1; j++) {
      const pcl::PointXYZ& p_g = search_cloud->points.at(nearest_nodes_index[j]);
      gtsam::Point3 gj(p_g.x, p_g.y, p_g.z);
      double weight =
          use_const_weight ? 1 : (1 - std::sqrt(nearest_nodes_sq_dist[j]) / d_max);
      weight_sum = weight_sum + weight;
      gtsam::Pose3 node_transform =
          values.at<gtsam::Pose3>(gtsam::Symbol(prefix, nearest_nodes_index[j]));
      gtsam::Point3 add =
          node_transform.rotation().rotate(vi - gj) + node_transform.translation();
      new_point = new_point + weight * add;
    }
    // Add back to new_vertices
    p_new.x = new_point.x() / weight_sum;
    p_new.y = new_point.y() / weight_sum;
    p_new.z = new_point.z() / weight_sum;
  }
}

/*! \brief Deform a points (i.e. the vertices of a mesh) based on the
 * controls points via deformation but also check timestamp of points
 * - original_points: set of points to deform
 * - stamps: timestamps of the points to deform
 * - prefix: a char to distinguish the type of control points
 * - control_points: original positions of the control points. In the case of
 * mesh vertices, these are the original positions of the simplified mesh.
 * - control_point_stamps: timestamps of the control points
 * - values: key-value pairs. Where each key should be gtsam::Symbol(prefix,
 * idx-in-control-points) from the previous two arguments.
 * - k: how many nearby nodes to use to adjust new position of vertices
 * - tol_t: time (in seconds) minimum difference in time that a control point
 * can be used for interpolation
 */
template <class point_type>
void deformPointsWithTimeCheck(pcl::PointCloud<point_type>& new_points,
                               std::vector<std::set<size_t>>& control_point_map,
                               const pcl::PointCloud<point_type>& points,
                               const std::vector<ros::Time>& stamps,
                               const char& prefix,
                               const std::vector<gtsam::Point3>& control_points,
                               const std::vector<ros::Time>& control_point_stamps,
                               const gtsam::Values& values,
                               size_t k = 4,
                               double tol_t = 10.0,
                               const std::vector<size_t>* indices = nullptr) {
  typedef pcl::octree::OctreePointCloudSearch<pcl::PointXYZ> Octree;
  // Check if there are points to deform
  const size_t num_points = indices ? indices->size() : points.size();
  if (!num_points) {
    return;
  }

  // Cannot deform if no nodes in the deformation graph
  if (control_points.size() < k) {
    ROS_WARN("Not enough valid control points to deform points. ");
    return;
  }

  assert(points.size() == stamps.size());
  assert(points.size() == new_points.size());
  assert(control_points.size() == control_point_stamps.size());
  control_point_map.clear();

  // Build Octree
  Octree::Ptr search_octree(new Octree(1.0));
  pcl::PointCloud<pcl::PointXYZ>::Ptr search_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  search_octree->setInputCloud(search_cloud);

  // By doing this implicitly assuming control_point_stamps is increasing
  // TODO(yun) check this assumption
  size_t ctrl_pt_idx = 0;
  size_t lower_ctrl_pt_idx = 0;
  for (size_t point_index = 0; point_index < num_points; ++point_index) {
    const size_t ii = indices ? indices->at(point_index) : point_index;
    size_t num_ctrl_pts = search_octree->getLeafCount();
    // Add control points to octree until both
    // exceeds interpolate horizon and have enough points to deform
    while (ctrl_pt_idx < control_points.size() &&
           (control_point_stamps[ctrl_pt_idx].toSec() <= stamps[ii].toSec() + tol_t ||
            num_ctrl_pts < k + 1)) {
      const gtsam::Point3& position = control_points[ctrl_pt_idx];
      search_cloud->push_back(pcl::PointXYZ(position.x(), position.y(), position.z()));
      if (!values.exists(gtsam::Symbol(prefix, ctrl_pt_idx))) {
        ctrl_pt_idx++;
        continue;
      }
      search_octree->addPointFromCloud(search_cloud->points.size() - 1, nullptr);
      num_ctrl_pts++;
      ctrl_pt_idx++;
    }

    if (search_octree->getLeafCount() < k + 1) {
      ROS_ERROR("Not enough valid control points in octree to interpolate point. ");
      if (num_ctrl_pts > 1) {
        k = num_ctrl_pts - 1;
      } else {
        continue;
      }
    }

    const point_type& p = points[ii];
    point_type& p_new = new_points[ii];
    // search for k + 1 nearest nodes
    std::vector<std::pair<size_t, double>> nearest_nodes;
    const pcl::PointXYZ p_xyz(p.x, p.y, p.z);
    const gtsam::Point3 vi(p.x, p.y, p.z);
    // Query octree
    std::vector<int> nn_index;
    std::vector<float> nn_sq_dist;
    search_octree->nearestKSearch(p_xyz, k + 1, nn_index, nn_sq_dist);

    // Calculate new point location from k points
    gtsam::Point3 new_point(0, 0, 0);
    double d_max = std::sqrt(nn_sq_dist[nn_index.size() - 1]);
    bool use_const_weight = false;
    if (std::sqrt(nn_sq_dist[0]) == d_max || d_max == 0) {
      use_const_weight = true;
    }
    double weight_sum = 0;
    control_point_map.push_back(std::set<size_t>());
    for (size_t j = 0; j < nn_index.size() - 1; j++) {
      const pcl::PointXYZ& p_g = search_cloud->points.at(nn_index[j]);
      gtsam::Point3 gj(p_g.x, p_g.y, p_g.z);
      double weight = use_const_weight ? 1 : (1 - std::sqrt(nn_sq_dist[j]) / d_max);
      weight_sum = weight_sum + weight;
      gtsam::Pose3 node_transform =
          values.at<gtsam::Pose3>(gtsam::Symbol(prefix, nn_index[j]));
      gtsam::Point3 add =
          node_transform.rotation().rotate(vi - gj) + node_transform.translation();
      new_point = new_point + weight * add;
      control_point_map.back().insert(nn_index[j]);
    }
    // Add back to new_vertices
    p_new.x = new_point.x() / weight_sum;
    p_new.y = new_point.y() / weight_sum;
    p_new.z = new_point.z() / weight_sum;

    size_t num_leaves = search_octree->getLeafCount();
    while (lower_ctrl_pt_idx < control_points.size() && num_leaves > k + 1 &&
           control_point_stamps[lower_ctrl_pt_idx].toSec() <
               stamps[ii].toSec() - tol_t) {
      if (!values.exists(gtsam::Symbol(prefix, lower_ctrl_pt_idx))) {
        lower_ctrl_pt_idx++;
        continue;
      }
      search_octree->deleteVoxelAtPoint(lower_ctrl_pt_idx);
      num_leaves--;
      lower_ctrl_pt_idx++;
    }
  }
}

}  // namespace kimera_pgmo
