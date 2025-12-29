find_package(GTest REQUIRED)
include(GoogleTest)
enable_testing()

# Unit tests
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
configure_file(test_config.h.in test_config.h)

add_executable(
  test_${PROJECT_NAME}
  pgmo_fixtures.cpp
  pgmo_unit_tests.cpp
  test_common_structs.cpp
  test_deformation_edge_factor.cpp
  test_deformation_graph.cpp
  test_delta_compression.cpp
  test_graph.cpp
  test_mesh_deformation.cpp
  test_mesh_delta.cpp
  test_mesh_frontend_interface.cpp
  test_mesh_io.cpp
  test_mesh_offset_info.cpp
  test_octree_compression.cpp
  test_pcl_utilities.cpp
  test_redundancy_checker.cpp
  test_traits.cpp
  test_block_compression.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)
target_include_directories(test_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR})
gtest_add_tests(TARGET test_${PROJECT_NAME})
