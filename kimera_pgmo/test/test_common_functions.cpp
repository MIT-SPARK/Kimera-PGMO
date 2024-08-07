/**
 * @file   test_common_functions.cpp
 * @brief  Unit-tests for some common functions
 * @author Yun Chang
 */

#include <gtsam/inference/Symbol.h>
#include <gtsam/slam/BetweenFactor.h>
#include <mesh_msgs/TriangleMesh.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PolygonMesh.h>
#include <pcl/conversions.h>
#include <ros/ros.h>

#include "gtest/gtest.h"
#include "kimera_pgmo/utils/CommonFunctions.h"
#include "kimera_pgmo/utils/CommonStructs.h"
#include "kimera_pgmo/utils/MeshIO.h"
#include "test_config.h"

namespace kimera_pgmo {

TEST(test_common_functions, PCLtoMeshMsg) {
  pcl::PolygonMeshPtr mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", mesh);

  // Convert to triangle mesh msg
  mesh_msgs::TriangleMesh triangle_mesh = PolygonMeshToTriangleMeshMsg(*mesh);

  // Convert back
  pcl::PolygonMesh new_mesh = TriangleMeshMsgToPolygonMesh(triangle_mesh);
  pcl::PointCloud<pcl::PointXYZRGBA> converted_vertices;
  pcl::fromPCLPointCloud2(new_mesh.cloud, converted_vertices);

  EXPECT_EQ(422, converted_vertices.points.size());
  EXPECT_EQ(-127, converted_vertices.points[0].z);
  EXPECT_EQ(5, converted_vertices.points[1].x);
  EXPECT_EQ(-1, converted_vertices.points[420].x);
  EXPECT_EQ(127, converted_vertices.points[421].z);
  EXPECT_EQ(mesh->polygons[0].vertices[2], new_mesh.polygons[0].vertices[2]);
  EXPECT_EQ(mesh->polygons[100].vertices[1], new_mesh.polygons[100].vertices[1]);
  EXPECT_EQ(mesh->polygons[839].vertices[0], new_mesh.polygons[839].vertices[0]);
}

TEST(test_common_functions, PolygonsEqual) {
  pcl::Vertices p0, p1, p2, p3;

  p0.vertices.push_back(0);
  p0.vertices.push_back(1);
  p0.vertices.push_back(2);

  p1.vertices.push_back(2);
  p1.vertices.push_back(0);
  p1.vertices.push_back(1);

  p2.vertices.push_back(0);
  p2.vertices.push_back(1);
  p2.vertices.push_back(3);

  p3.vertices.push_back(0);
  p3.vertices.push_back(1);
  p3.vertices.push_back(2);
  p3.vertices.push_back(4);

  EXPECT_TRUE(PolygonsEqual(p0, p0));
  EXPECT_TRUE(PolygonsEqual(p0, p1));
  EXPECT_FALSE(PolygonsEqual(p0, p2));
  EXPECT_FALSE(PolygonsEqual(p0, p3));
}

TEST(test_common_functions, RosPoseToGtsam) {
  geometry_msgs::Pose ros_pose;
  gtsam::Pose3 gtsam_pose = RosToGtsam(ros_pose);
  EXPECT_TRUE(gtsam::assert_equal(gtsam::Pose3(), gtsam_pose));

  ros_pose.position.x = 1;
  ros_pose.position.z = 100;
  ros_pose.orientation.y = 0.707;
  ros_pose.orientation.w = 0.707;
  gtsam_pose = RosToGtsam(ros_pose);
  EXPECT_TRUE(gtsam::assert_equal(
      gtsam::Pose3(gtsam::Rot3(0.707, 0, 0.707, 0), gtsam::Point3(1, 0, 100)),
      gtsam_pose));
}

TEST(test_common_functions, GtsamPoseToRos) {
  gtsam::Pose3 gtsam_pose;
  geometry_msgs::Pose ros_pose = GtsamToRos(gtsam_pose);
  EXPECT_EQ(0, ros_pose.position.x);
  EXPECT_EQ(0, ros_pose.position.y);
  EXPECT_EQ(0, ros_pose.position.z);
  EXPECT_EQ(0, ros_pose.orientation.x);
  EXPECT_EQ(0, ros_pose.orientation.y);
  EXPECT_EQ(0, ros_pose.orientation.z);
  EXPECT_EQ(1, ros_pose.orientation.w);

  gtsam_pose = gtsam::Pose3(gtsam::Rot3(0.707, 0, 0.707, 0), gtsam::Point3(1, 0, 100));
  ros_pose = GtsamToRos(gtsam_pose);
  EXPECT_EQ(1, ros_pose.position.x);
  EXPECT_EQ(0, ros_pose.position.y);
  EXPECT_EQ(100, ros_pose.position.z);
  EXPECT_NEAR(0, ros_pose.orientation.x, 0.001);
  EXPECT_NEAR(0.707, ros_pose.orientation.y, 0.001);
  EXPECT_NEAR(0, ros_pose.orientation.z, 0.001);
  EXPECT_NEAR(0.707, ros_pose.orientation.w, 0.001);
}

TEST(test_common_functions, PclPointToGtsam) {
  pcl::PointXYZRGBA point_rgb;
  point_rgb.x = 1.0;
  point_rgb.y = 2.0;
  point_rgb.z = -0.3;
  point_rgb.r = 22;
  point_rgb.g = 132;
  point_rgb.b = 255;
  point_rgb.a = 255;
  gtsam::Point3 gtsam_rgb_pt = PclToGtsam<pcl::PointXYZRGBA>(point_rgb);

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(point_rgb.x, point_rgb.y, point_rgb.z),
                                  gtsam_rgb_pt));

  pcl::PointXYZ point;
  point.x = -1.0;
  point.y = 20.0;
  point.z = 0.35;
  gtsam::Point3 gtsam_pt = PclToGtsam<pcl::PointXYZ>(point);

  EXPECT_TRUE(gtsam::assert_equal(gtsam::Point3(point.x, point.y, point.z), gtsam_pt));
}

// Combine Meshes
TEST(test_common_functions, CombineMeshesNoCheck) {
  pcl::PolygonMeshPtr sphere_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", sphere_mesh);

  // Test combining sphere mesh with itself and not check for duplicates
  pcl::PolygonMesh combined = CombineMeshes(*sphere_mesh, *sphere_mesh, false);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(combined.cloud, cloud);

  pcl::PointXYZ first_point(0, 0, -127);
  pcl::PointXYZ last_point(0, 0, 127);

  // Check number of surfaces
  EXPECT_EQ(size_t(1680), combined.polygons.size());
  EXPECT_EQ(size_t(844), cloud.points.size());

  // Check polygons parsed correctly
  EXPECT_EQ(0, combined.polygons[0].vertices[0]);
  EXPECT_EQ(2, combined.polygons[0].vertices[2]);
  EXPECT_EQ(842, combined.polygons[1679].vertices[0]);
  EXPECT_EQ(813, combined.polygons[1679].vertices[2]);

  // Check vertices parsed correctly
  EXPECT_EQ(-127, cloud.points[0].z);
  EXPECT_EQ(127, cloud.points[843].z);
}

TEST(test_common_functions, CombineMeshesCheck) {
  pcl::PolygonMeshPtr sphere_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", sphere_mesh);

  // Test combining sphere mesh with itself and check for duplicates
  pcl::PolygonMesh combined = CombineMeshes(*sphere_mesh, *sphere_mesh, true);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(combined.cloud, cloud);

  pcl::PointXYZ first_point(0, 0, -127);
  pcl::PointXYZ last_point(0, 0, 127);

  // Check number of surfaces
  EXPECT_EQ(size_t(840), combined.polygons.size());
  EXPECT_EQ(size_t(422), cloud.points.size());

  // Check polygons parsed correctly
  EXPECT_EQ(0, combined.polygons[0].vertices[0]);
  EXPECT_EQ(2, combined.polygons[0].vertices[2]);
  EXPECT_EQ(420, combined.polygons[839].vertices[0]);
  EXPECT_EQ(391, combined.polygons[839].vertices[2]);

  // Check vertices parsed correctly
  EXPECT_EQ(-127, cloud.points[0].z);
  EXPECT_EQ(127, cloud.points[421].z);
}

TEST(test_common_functions, CombineMeshesIndices) {
  pcl::PolygonMeshPtr sphere_mesh(new pcl::PolygonMesh());
  ReadMeshFromPly(std::string(DATASET_PATH) + "/sphere.ply", sphere_mesh);

  // Test combining sphere with itself, check for duplicates with given vertices
  std::vector<size_t> ind_to_check{0, 1, 2, 3, 4, 5, 6, 100, 200, 300, 421};
  std::vector<size_t> new_indices;
  pcl::PolygonMesh combined =
      CombineMeshes(*sphere_mesh, *sphere_mesh, ind_to_check, &new_indices);
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(combined.cloud, cloud);

  pcl::PointXYZ first_point(0, 0, -127);
  pcl::PointXYZ last_point(-1, 26, 124);

  // Check number of surfaces
  EXPECT_EQ(size_t(1675), combined.polygons.size());
  EXPECT_EQ(size_t(833), cloud.points.size());

  // Check polygons parsed correctly
  EXPECT_EQ(0, combined.polygons[0].vertices[0]);
  EXPECT_EQ(2, combined.polygons[0].vertices[2]);
  EXPECT_EQ(832, combined.polygons[1674].vertices[0]);
  EXPECT_EQ(803, combined.polygons[1674].vertices[2]);

  // Check vertices parsed correctly
  EXPECT_EQ(-127, cloud.points[0].z);
  EXPECT_EQ(124, cloud.points[832].z);

  // Check the indices of added mesh
  EXPECT_EQ(size_t(422), new_indices.size());
  EXPECT_EQ(0, new_indices[0]);
  EXPECT_EQ(422, new_indices[7]);
  EXPECT_EQ(832, new_indices[420]);
  EXPECT_EQ(421, new_indices[421]);
}

TEST(test_common_functions, PolygonEquality) {
  pcl::Vertices poly_1, poly_2, poly_3, poly_4;

  poly_1.vertices = std::vector<uint32_t>{0, 1, 2};
  poly_2.vertices = std::vector<uint32_t>{2, 0, 1};
  poly_3.vertices = std::vector<uint32_t>{1, 0, 2};
  poly_4.vertices = std::vector<uint32_t>{3, 4, 5};

  EXPECT_TRUE(PolygonsEqual(poly_1, poly_1));
  EXPECT_TRUE(PolygonsEqual(poly_1, poly_2));
  EXPECT_FALSE(PolygonsEqual(poly_1, poly_3));
  EXPECT_FALSE(PolygonsEqual(poly_1, poly_4));
  EXPECT_TRUE(PolygonsEqual(poly_4, poly_4));
}

// GTSAM graph to ROS
TEST(test_common_functions, GtsamGraphToRos) {
  ros::Time::init();

  static const gtsam::SharedNoiseModel& noise =
      gtsam::noiseModel::Isotropic::Variance(6, 0.01);
  gtsam::NonlinearFactorGraph nfg;
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 0), gtsam::Symbol('a', 1), gtsam::Pose3(), noise));
  nfg.add(gtsam::BetweenFactor<gtsam::Pose3>(
      gtsam::Symbol('a', 1),
      gtsam::Symbol('a', 2),
      gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 1, 1)),
      noise));

  gtsam::Values val;
  std::map<size_t, std::vector<Timestamp> > time_stamps;
  time_stamps[0] = std::vector<Timestamp>{};
  val.insert(gtsam::Symbol('a', 0), gtsam::Pose3());
  time_stamps[0].push_back(stampFromSec(0.01));
  val.insert(gtsam::Symbol('a', 1), gtsam::Pose3());
  time_stamps[0].push_back(stampFromSec(0.02));
  val.insert(gtsam::Symbol('a', 2),
             gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 1, 1)));
  time_stamps[0].push_back(stampFromSec(0.03));

  const GraphMsgPtr& pose_graph_ptr = GtsamGraphToRos(nfg, val, time_stamps);

  // Check edges
  EXPECT_EQ(size_t(2), pose_graph_ptr->edges.size());
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(), RosToGtsam(pose_graph_ptr->edges[0].pose)));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 1, 1)),
                          RosToGtsam(pose_graph_ptr->edges[1].pose)));

  EXPECT_EQ(0, pose_graph_ptr->edges[0].robot_from);
  EXPECT_EQ(0, pose_graph_ptr->edges[0].robot_to);
  EXPECT_EQ(0, pose_graph_ptr->edges[0].key_from);
  EXPECT_EQ(1, pose_graph_ptr->edges[0].key_to);
  EXPECT_EQ(0, pose_graph_ptr->edges[1].robot_from);
  EXPECT_EQ(0, pose_graph_ptr->edges[1].robot_to);
  EXPECT_EQ(1, pose_graph_ptr->edges[1].key_from);
  EXPECT_EQ(2, pose_graph_ptr->edges[1].key_to);
  EXPECT_EQ("world", pose_graph_ptr->edges[1].header.frame_id);

  // Check nodes
  EXPECT_EQ(size_t(3), pose_graph_ptr->nodes.size());
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(), RosToGtsam(pose_graph_ptr->nodes[0].pose)));
  EXPECT_TRUE(
      gtsam::assert_equal(gtsam::Pose3(gtsam::Rot3(0, 1, 0, 0), gtsam::Point3(1, 1, 1)),
                          RosToGtsam(pose_graph_ptr->nodes[2].pose)));
  EXPECT_EQ(0, pose_graph_ptr->nodes[0].robot_id);
  EXPECT_EQ(0, pose_graph_ptr->nodes[2].robot_id);
  EXPECT_EQ(0, pose_graph_ptr->nodes[0].key);
  EXPECT_EQ(1, pose_graph_ptr->nodes[1].key);
  EXPECT_EQ(2, pose_graph_ptr->nodes[2].key);
}

TEST(test_common_functions, MeshSurfaceExist) {
  std::map<size_t, std::vector<size_t> > adj_surfaces;

  pcl::Vertices poly_1, poly_2, poly_3, poly_4;
  poly_1.vertices = std::vector<uint32_t>{0, 1, 2};
  poly_2.vertices = std::vector<uint32_t>{0, 2, 3};
  poly_3.vertices = std::vector<uint32_t>{0, 5, 1};
  poly_4.vertices = std::vector<uint32_t>{1, 4, 2};
  std::vector<pcl::Vertices> surfaces{poly_1, poly_2, poly_3, poly_4};

  adj_surfaces[0] = std::vector<size_t>{0, 1};
  adj_surfaces[1] = std::vector<size_t>{0, 3};
  adj_surfaces[2] = std::vector<size_t>{0, 1, 3};
  adj_surfaces[3] = std::vector<size_t>{1};
  adj_surfaces[4] = std::vector<size_t>{3};
  EXPECT_FALSE(SurfaceExists(poly_3, adj_surfaces, surfaces));

  adj_surfaces[0].push_back(2);
  adj_surfaces[1].push_back(2);
  adj_surfaces[5] = std::vector<size_t>{2};
  EXPECT_TRUE(SurfaceExists(poly_3, adj_surfaces, surfaces));
}

}  // namespace kimera_pgmo
