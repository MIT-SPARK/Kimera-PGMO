cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo)

SET(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED COMPONENTS
  date_time
  serialization
  thread
  filesystem
  system
  regex
  timer
  chrono
)

find_package(PCL REQUIRED COMPONENTS
  common 
  io 
  geometry
  kdtree
  octree
)

find_package(KimeraRPGO REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
src/compression/MeshCompression.cpp
src/compression/OctreeCompression.cpp
src/compression/VoxbloxCompression.cpp
src/compression/VoxelClearingCompression.cpp
src/utils/CommonFunctions.cpp
src/utils/CommonStructs.cpp
src/utils/VoxbloxUtils.cpp
src/DeformationGraph.cpp
src/DeformationGraphIo.cpp
src/KimeraPgmo.cpp
src/MeshFrontend.cpp
src/KimeraPgmoInterface.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  KimeraRPGO
  gtsam
  tbb
)

cs_add_executable(kimera_pgmo_node
  src/kimera_pgmo_node.cpp
)
target_link_libraries(kimera_pgmo_node ${PROJECT_NAME})

cs_add_executable(mesh_frontend_node
  src/mesh_frontend_node.cpp
)
target_link_libraries(mesh_frontend_node ${PROJECT_NAME})


# Unit tests 
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
configure_file(test/test_config.h.in test/test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

catkin_add_gtest(${PROJECT_NAME}-test_common_functions test/test_common_functions.cpp)
target_link_libraries(${PROJECT_NAME}-test_common_functions ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_deformation_edge_factor test/test_deformation_edge_factor.cpp)
target_link_libraries(${PROJECT_NAME}-test_deformation_edge_factor ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_deformation_graph test/test_deformation_graph.cpp)
target_link_libraries(${PROJECT_NAME}-test_deformation_graph ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_graph test/test_graph.cpp)
target_link_libraries(${PROJECT_NAME}-test_graph ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_octree_compression test/test_octree_compression.cpp)
target_link_libraries(${PROJECT_NAME}-test_octree_compression ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_voxblox_compression test/test_voxblox_compression.cpp)
target_link_libraries(${PROJECT_NAME}-test_voxblox_compression ${PROJECT_NAME})

catkin_add_gtest(${PROJECT_NAME}-test_voxel_clearing_compression test/test_voxel_clearing_compression.cpp)
target_link_libraries(${PROJECT_NAME}-test_voxel_clearing_compression ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_mesh_frontend test/test_mesh_frontend.test test/test_mesh_frontend.cpp)
target_link_libraries(${PROJECT_NAME}-test_mesh_frontend ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_pgmo test/test_kimera_pgmo.test test/test_kimera_pgmo.cpp)
target_link_libraries(${PROJECT_NAME}-test_kimera_pgmo ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_dpgmo test/test_kimera_dpgmo.test test/test_kimera_dpgmo.cpp)
target_link_libraries(${PROJECT_NAME}-test_kimera_dpgmo ${PROJECT_NAME})

cs_install()

cs_export()


