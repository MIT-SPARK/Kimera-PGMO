cmake_minimum_required(VERSION 3.22)
project(kimera_pgmo_msgs)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(
  ${PROJECT_NAME}
  msg/AbsolutePoseStamped.msg
  msg/Mesh.msg
  msg/MeshDelta.msg
  msg/TriangleIndices.msg
  srv/LoadGraphMesh.srv
  srv/MeshDeltaQuery.srv
  DEPENDENCIES
  builtin_interfaces
  geometry_msgs
  std_msgs
)

ament_package()
