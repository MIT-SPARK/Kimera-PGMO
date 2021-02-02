cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo)

SET(CMAKE_BUILD_TYPE Release)

find_package(catkin_simple REQUIRED)

find_package(Boost REQUIRED
  date_time
  serialization
  thread
  filesystem
  system
  regex
  timer
  chrono
)

find_package(KimeraRPGO REQUIRED)

catkin_simple()

cs_add_library(${PROJECT_NAME}
src/compression/OctreeCompression.cpp
src/utils/CommonFunctions.cpp
src/utils/CommonStructs.cpp
src/utils/VoxbloxUtils.cpp
src/utils/tinyply.cpp
src/DeformationGraph.cpp
src/KimeraPgmo.cpp
src/VoxbloxProcessing.cpp
src/KimeraPgmoMulti.cpp
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

cs_add_executable(voxblox_processing_node
  src/voxblox_processing_node.cpp
)
target_link_libraries(voxblox_processing_node ${PROJECT_NAME})

cs_add_executable(load_mesh
  src/load_mesh.cpp
)
target_link_libraries(load_mesh ${PROJECT_NAME})


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

add_rostest_gtest(${PROJECT_NAME}-test_voxblox_processing test/test_voxblox_processing.test test/test_voxblox_processing.cpp)
target_link_libraries(${PROJECT_NAME}-test_voxblox_processing ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_pgmo test/test_kimera_pgmo.test test/test_kimera_pgmo.cpp)
target_link_libraries(${PROJECT_NAME}-test_kimera_pgmo ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_pgmo_multi test/test_kimera_pgmo_multi.test test/test_kimera_pgmo_multi.cpp)
target_link_libraries(${PROJECT_NAME}-test_kimera_pgmo_multi ${PROJECT_NAME})

cs_install()

cs_export()


