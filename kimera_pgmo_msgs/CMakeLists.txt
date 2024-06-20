cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs
                                        geometry_msgs pose_graph_tools_msgs)

add_message_files(FILES AbsolutePoseStamped.msg KimeraPgmoMesh.msg
                  KimeraPgmoMeshDelta.msg TriangleIndices.msg)
add_service_files(FILES LoadGraphMesh.srv RequestMeshFactors.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs pose_graph_tools_msgs)

catkin_package(
  CATKIN_DEPENDS
  message_runtime
  std_msgs
  geometry_msgs
  pose_graph_tools_msgs
  DEPENDS
  INCLUDE_DIRS
  LIBRARIES)
