cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo_ros)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-sign-compare -Wno-unused-parameter)

find_package(config_utilities REQUIRED)
find_package(kimera_pgmo REQUIRED)
find_package(
  catkin REQUIRED
  COMPONENTS kimera_pgmo_msgs
             nav_msgs
             pose_graph_tools_msgs
             pose_graph_tools_ros
             roscpp
             tf2_ros
             visualization_msgs)

catkin_package(
  CATKIN_DEPENDS
  kimera_pgmo_msgs
  nav_msgs
  pose_graph_tools_msgs
  pose_graph_tools_ros
  roscpp
  tf2_ros
  visualization_msgs
  DEPENDS
  kimera_pgmo
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME})

add_library(${PROJECT_NAME}  
  src/conversion/gtsam_conversions.cpp
  src/kimera_pgmo.cpp
  src/conversion/mesh_conversion.cpp
  src/conversion/mesh_delta_conversion.cpp
  src/mesh_frontend.cpp
  src/visualization_functions.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${catkin_LIBRARIES} kimera_pgmo::kimera_pgmo 
  PRIVATE config_utilities::config_utilities)

add_executable(kimera_pgmo_node app/kimera_pgmo_node.cpp)
target_link_libraries(kimera_pgmo_node ${PROJECT_NAME}
                      config_utilities::config_utilities)

add_executable(mesh_frontend_node app/mesh_frontend_node.cpp)
target_link_libraries(mesh_frontend_node ${PROJECT_NAME}
                      config_utilities::config_utilities)

add_executable(mesh_publisher_node app/mesh_publisher_node.cpp)
target_link_libraries(mesh_publisher_node ${PROJECT_NAME}
                      config_utilities::config_utilities)

add_executable(mesh_trajectory_deformer app/mesh_trajectory_deformer.cpp)
target_link_libraries(mesh_trajectory_deformer ${PROJECT_NAME}
                      config_utilities::config_utilities)

# Optionally build the voxblox frontend if voxblox_msgs is present.
find_package(voxblox_msgs QUIET)
if(voxblox_msgs_FOUND)
  message(STATUS "Building with voxblox_msgs support")
  add_executable(voxblox_mesh_frontend_node 
    app/voxblox_mesh_frontend_node.cpp 
    src/conversion/voxblox_msg_conversion.cpp
    src/voxblox_mesh_frontend.cpp
    src/voxblox_msg_interface.cpp
  )
  target_link_libraries(voxblox_mesh_frontend_node 
    PUBLIC ${PROJECT_NAME}
    PRIVATE config_utilities::config_utilities ${voxblox_msgs_LIBRARIES})
endif()

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

install(
  TARGETS ${PROJECT_NAME} kimera_pgmo_node mesh_frontend_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
