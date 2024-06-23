if(NOT TARGET gtest)
  include(FetchContent)
  FetchContent_Declare(
    googletest GIT_REPOSITORY https://github.com/google/googletest.git GIT_TAG v1.13.0
  )

  FetchContent_GetProperties(googletest)
  if(NOT googletest_POPULATED)
    FetchContent_Populate(googletest)
  endif()

  set(BUILD_SHARED_LIBS_THIS ${BUILD_SHARED_LIBS})
  set(BUILD_SHARED_LIBS OFF CACHE INTERNAL "")
  add_subdirectory(${googletest_SOURCE_DIR} ${googletest_BINARY_DIR} EXCLUDE_FROM_ALL)
  set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_THIS} CACHE BOOL "")
endif()

include(GoogleTest)
enable_testing()

# Unit tests
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/data")
configure_file(test_config.h.in test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR})

add_executable(
  test_${PROJECT_NAME}
  pgmo_fixtures.cpp
  pgmo_unit_tests.cpp
  test_common_functions.cpp
  test_common_structs.cpp
  test_deformation_edge_factor.cpp
  test_deformation_graph.cpp
  test_delta_compression.cpp
  test_graph.cpp
  test_mesh_deformation.cpp
  test_mesh_delta.cpp
  test_mesh_frontend_interface.cpp
  test_mesh_io.cpp
  test_octree_compression.cpp
  test_traits.cpp
  test_block_compression.cpp
  test_voxel_clearing_compression.cpp)
target_link_libraries(test_${PROJECT_NAME} ${PROJECT_NAME} GTest::gtest_main)
gtest_add_tests(TARGET test_${PROJECT_NAME})
