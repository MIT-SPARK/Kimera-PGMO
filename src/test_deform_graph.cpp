#include <string>

#include <pcl/PolygonMesh.h>
#include <pcl_msgs/PolygonMesh.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include "mesher_mapper/CommonFunctions.h"
#include "mesher_mapper/DeformationGraph.h"

using mesher_mapper::Vertex;
using mesher_mapper::Vertices;

class test_deformation_graph {
 public:
  test_deformation_graph(const ros::NodeHandle& n, pcl::PolygonMeshPtr new_mesh)
      : deformation_graph_(new_mesh) {
    original_mesh_ = *new_mesh;

    ros::NodeHandle nl(n);
    mesh_pub_ = nl.advertise<pcl_msgs::PolygonMesh>("deformed_mesh", 10, true);
    input_sub_ = nl.subscribe<std_msgs::String>(
        "loop_closures", 1, &test_deformation_graph::InputCallback, this);

    // Do a beginning publish
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    pcl_msgs::PolygonMesh new_msg =
        mesher_mapper::ConstructPolygonMeshMsg(*new_mesh, header);
    mesh_pub_.publish(new_msg);
  }

 private:
  mesher_mapper::DeformationGraph deformation_graph_;
  pcl::PolygonMesh original_mesh_;

  ros::Publisher mesh_pub_;
  ros::Subscriber input_sub_;

  void InputCallback(const std_msgs::String::ConstPtr& input_msg) {
    Vertices vertices;
    std::string str_key = "";
    std::string word = "";
    for (auto x : input_msg->data) {
      if (x == ' ') {
        Vertex v = std::stoi(word);
        vertices.push_back(v);
        word = "";
      } else {
        word = word + x;
      }
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr new_point_positions(
        new pcl::PointCloud<pcl::PointXYZ>);
    deformation_graph_.loopClose(vertices[0], vertices[1], new_point_positions);

    // Generate new polygon mesh msg
    pcl::PolygonMesh new_mesh;
    new_mesh.polygons = original_mesh_.polygons;
    pcl::toPCLPointCloud2(*new_point_positions, new_mesh.cloud);

    std_msgs::Header header;
    header.stamp = ros::Time::now();
    pcl_msgs::PolygonMesh new_msg =
        mesher_mapper::ConstructPolygonMeshMsg(new_mesh, header);
    mesh_pub_.publish(new_msg);
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_deform_graph");
  // ros::NodeHandle n;

  // Load input mesh
  pcl::PolygonMeshPtr input_mesh(new pcl::PolygonMesh());
  mesher_mapper::ReadMeshFromPly(argv[1], input_mesh);

  // test_deformation_graph test(n, input_mesh);

  // ros::spin();

  return 0;
}
