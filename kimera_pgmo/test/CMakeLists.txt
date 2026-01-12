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
  suites/compression/test_block_compression.cpp
  suites/compression/test_delta_compression.cpp
  suites/compression/test_octree_compression.cpp
  suites/compression/test_redundancy_checker.cpp
  suites/utils/test_common_functions.cpp
  suites/utils/test_common_structs.cpp
  suites/utils/test_graph.cpp
  suites/utils/test_mesh_io.cpp
  suites/utils/test_pcl_utilities.cpp
  suites/utils/test_range_generator.cpp
  suites/test_deformation_edge_factor.cpp
  suites/test_deformation_graph.cpp
  suites/test_mesh_deformation.cpp
  suites/test_mesh_delta.cpp
  suites/test_mesh_frontend_interface.cpp
  suites/test_mesh_offset_info.cpp
  suites/test_mesh_traits.cpp
  suites/test_mesh_types.cpp
  suites/test_pcl_mesh_traits.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)
target_include_directories(
  test_${PROJECT_NAME} PRIVATE ${CMAKE_CURRENT_BINARY_DIR}
                               ${CMAKE_CURRENT_SOURCE_DIR})
gtest_add_tests(TARGET test_${PROJECT_NAME})
