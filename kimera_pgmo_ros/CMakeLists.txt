cmake_minimum_required(VERSION 3.22.1)
project(kimera_pgmo_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(ament_cmake REQUIRED)
find_package(kimera_pgmo REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pose_graph_tools_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(
  ${PROJECT_NAME}
  src/conversion/gtsam.cpp src/kimera_pgmo.cpp src/conversion/mesh.cpp
  src/conversion/mesh_delta.cpp src/mesh_frontend.cpp
  src/visualization_functions.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PRIVATE config_utilities::config_utilities
                                              tf2_eigen::tf2_eigen)
ament_target_dependencies(
  ${PROJECT_NAME}
  PUBLIC
  kimera_pgmo
  kimera_pgmo_msgs
  nav_msgs
  pose_graph_tools_ros
  rclcpp
  std_srvs
  tf2_ros
  visualization_msgs)

add_executable(kimera_pgmo_node app/kimera_pgmo_node.cpp)
target_link_libraries(kimera_pgmo_node ${PROJECT_NAME})

add_executable(mesh_frontend_node app/mesh_frontend_node.cpp)
target_link_libraries(mesh_frontend_node ${PROJECT_NAME})

add_executable(mesh_publisher_node app/mesh_publisher_node.cpp)
target_link_libraries(mesh_publisher_node ${PROJECT_NAME})

if(BUILD_TESTING)
  add_subdirectory(test)
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
# NOTE(nathan) this avoids executable targets being exported and found during
# find_package(kimera_pgmo_ros ...)
install(TARGETS kimera_pgmo_node mesh_frontend_node mesh_publisher_node
        RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(
  kimera_pgmo
  kimera_pgmo_msgs
  nav_msgs
  pose_graph_tools_ros
  rclcpp
  std_srvs
  tf2_ros
  visualization_msgs)
ament_package()
