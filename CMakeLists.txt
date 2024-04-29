cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-sign-compare -Wno-unused-parameter)

find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(KimeraRPGO REQUIRED)

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             mesh_msgs
             message_generation
             nav_msgs
             pose_graph_tools_msgs
             pose_graph_tools_ros
             roscpp
             std_msgs
             tf2_ros
             visualization_msgs
             voxblox
             voxblox_msgs)

set(PCL_FIND_QUIETLY TRUE)
find_package(PCL REQUIRED COMPONENTS common geometry kdtree octree)

add_message_files(FILES AbsolutePoseStamped.msg KimeraPgmoMesh.msg
                  KimeraPgmoMeshDelta.msg TriangleIndices.msg)
add_service_files(FILES LoadGraphMesh.srv RequestMeshFactors.srv)
generate_messages(DEPENDENCIES std_msgs geometry_msgs pose_graph_tools_msgs)

catkin_package(
  CATKIN_DEPENDS
  geometry_msgs
  mesh_msgs
  message_runtime
  nav_msgs
  pose_graph_tools_msgs
  pose_graph_tools_ros
  roscpp
  std_msgs
  tf2_ros
  visualization_msgs
  voxblox
  voxblox_msgs
  DEPENDS
  KimeraRPGO
  PCL
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_NAME}
  gtsam)

add_library(
  ${PROJECT_NAME}
  src/compression/MeshCompression.cpp
  src/compression/DeltaCompression.cpp
  src/compression/OctreeCompression.cpp
  src/compression/VoxelClearingCompression.cpp
  src/compression/VoxbloxCompression.cpp
  src/utils/CommonFunctions.cpp
  src/utils/CommonStructs.cpp
  src/utils/MeshIO.cpp
  src/utils/RangeGenerator.cpp
  src/utils/TriangleMeshConversion.cpp
  src/utils/VoxbloxMeshInterface.cpp
  src/utils/VoxbloxMsgInterface.cpp
  src/utils/VoxbloxUtils.cpp
  src/DeformationGraph.cpp
  src/DeformationGraphIo.cpp
  src/KimeraPgmo.cpp
  src/MeshDeformation.cpp
  src/MeshDelta.cpp
  src/MeshFrontend.cpp
  src/MeshFrontendInterface.cpp
  src/MeshTraits.cpp
  src/KimeraPgmoInterface.cpp)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS}
                 ${catkin_EXPORTED_TARGETS})
target_include_directories(
  ${PROJECT_NAME} PUBLIC include ${catkin_INCLUDE_DIRS} ${PCL_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES} ${PCL_LIBRARIES} Eigen3::Eigen
                         KimeraRPGO gtsam)

add_executable(kimera_pgmo_node src/kimera_pgmo_node.cpp)
target_link_libraries(kimera_pgmo_node ${PROJECT_NAME})

add_executable(mesh_frontend_node src/mesh_frontend_node.cpp)
target_link_libraries(mesh_frontend_node ${PROJECT_NAME})

add_executable(mesh_trajectory_deformer src/mesh_trajectory_deformer.cpp)
target_link_libraries(mesh_trajectory_deformer ${PROJECT_NAME})

if(CATKIN_ENABLE_TESTING)
  add_subdirectory(test)
endif()

install(
  TARGETS ${PROJECT_NAME} kimera_pgmo_node mesh_frontend_node
          mesh_trajectory_deformer
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY launch/
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch)
