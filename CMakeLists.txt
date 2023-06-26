cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo)

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
src/compression/DeltaCompression.cpp
src/compression/OctreeCompression.cpp
src/compression/VoxelClearingCompression.cpp
src/compression/VoxbloxCompression.cpp
src/utils/CommonFunctions.cpp
src/utils/CommonStructs.cpp
src/utils/VoxbloxMeshInterface.cpp
src/utils/VoxbloxMsgInterface.cpp
src/utils/VoxbloxUtils.cpp
src/DeformationGraph.cpp
src/DeformationGraphIo.cpp
src/KimeraPgmo.cpp
src/MeshDelta.cpp
src/MeshFrontend.cpp
src/MeshFrontendInterface.cpp
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

cs_add_executable(mesh_trajectory_deformer
  src/mesh_trajectory_deformer.cpp
)
target_link_libraries(mesh_trajectory_deformer ${PROJECT_NAME})


# Unit tests
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
configure_file(test/test_config.h.in test/test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)

catkin_add_gtest(${PROJECT_NAME}-test
test/pgmo_unit_tests.cpp
test/test_common_functions.cpp
test/test_deformation_edge_factor.cpp
test/test_deformation_graph.cpp
test/test_graph.cpp
test/test_mesh_delta.cpp
test/test_delta_compression.cpp
test/test_voxblox_compression.cpp
test/test_voxel_clearing_compression.cpp
test/test_octree_compression.cpp
)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_mesh_frontend test/test_mesh_frontend.test test/test_mesh_frontend.cpp)
target_link_libraries(${PROJECT_NAME}-test_mesh_frontend ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_pgmo test/test_kimera_pgmo.test test/test_kimera_pgmo.cpp)
target_link_libraries(${PROJECT_NAME}-test_kimera_pgmo ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_dpgmo test/test_kimera_dpgmo.test test/test_kimera_dpgmo.cpp)
target_link_libraries(${PROJECT_NAME}-test_kimera_dpgmo ${PROJECT_NAME})

cs_install()

cs_export()
