find_package(rostest REQUIRED)

# Unit tests
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
configure_file(test_config.h.in test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

catkin_add_gtest(
  ${PROJECT_NAME}-test
  pgmo_unit_tests.cpp
  test_common_structs.cpp
  test_common_functions.cpp
  test_deformation_edge_factor.cpp
  test_deformation_graph.cpp
  test_graph.cpp
  test_mesh_deformation.cpp
  test_mesh_delta.cpp
  test_mesh_io.cpp
  test_delta_compression.cpp
  test_voxblox_compression.cpp
  test_voxel_clearing_compression.cpp
  test_octree_compression.cpp
  test_traits.cpp)
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_mesh_frontend test_mesh_frontend.test
                  test_mesh_frontend.cpp)
target_link_libraries(${PROJECT_NAME}-test_mesh_frontend ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_pgmo test_kimera_pgmo.test
                  test_kimera_pgmo.cpp)
target_link_libraries(${PROJECT_NAME}-test_kimera_pgmo ${PROJECT_NAME})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_dpgmo test_kimera_dpgmo.test
                  test_kimera_dpgmo.cpp)
target_link_libraries(${PROJECT_NAME}-test_kimera_dpgmo ${PROJECT_NAME})
