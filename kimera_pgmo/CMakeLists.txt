cmake_minimum_required(VERSION 3.5)
project(kimera_pgmo VERSION 0.0.1)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra -Wno-sign-compare -Wno-unused-parameter)

option(KIMERA_PGMO_ENABLE_TESTS "Build unit tets" ON)
option(BUILD_SHARED_LIBS "Build shared libraries" ON)

find_package(config_utilities REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(GTSAM REQUIRED)
find_package(KimeraRPGO REQUIRED)
find_package(PCL REQUIRED COMPONENTS common geometry kdtree octree)
find_package(pose_graph_tools REQUIRED)
find_package(spatial_hash REQUIRED)

include(GNUInstallDirs)
include(CMakePackageConfigHelpers)

add_library(
  ${PROJECT_NAME}
  src/compression/mesh_compression.cpp
  src/compression/delta_compression.cpp
  src/compression/octree_compression.cpp
  src/compression/voxel_clearing_compression.cpp
  src/compression/block_compression.cpp
  src/utils/common_functions.cpp
  src/utils/common_structs.cpp
  src/utils/logging.cpp
  src/utils/mesh_io.cpp
  src/utils/range_generator.cpp
  src/deformation_graph.cpp
  src/deformation_graph_io.cpp
  src/mesh_deformation.cpp
  src/mesh_delta.cpp
  src/mesh_frontend_interface.cpp
  src/mesh_traits.cpp
  src/kimera_pgmo_interface.cpp
  src/sparse_keyframe.cpp)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC $<INSTALL_INTERFACE:include>
         $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
         ${PCL_INCLUDE_DIRS})
target_link_libraries(
  ${PROJECT_NAME}
  PUBLIC ${PCL_LIBRARIES} Eigen3::Eigen KimeraRPGO gtsam
         pose_graph_tools::pose_graph_tools spatial_hash::spatial_hash
  PRIVATE "$<BUILD_INTERFACE:config_utilities::config_utilities>")
set_property(TARGET ${PROJECT_NAME} PROPERTY POSITION_INDEPENDENT_CODE ON)
add_library(kimera_pgmo::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

if(KIMERA_PGMO_ENABLE_TESTS)
  enable_testing()
  add_subdirectory(test)
endif()

install(
  TARGETS ${PROJECT_NAME}
  EXPORT kimera_pgmo-targets
  ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
  LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})
install(
  EXPORT kimera_pgmo-targets
  FILE kimera_pgmoTargets.cmake
  NAMESPACE kimera_pgmo::
  DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kimera_pgmo)

configure_package_config_file(
  ${CMAKE_CURRENT_LIST_DIR}/cmake/kimera_pgmoConfig.cmake.in
  ${CMAKE_CURRENT_BINARY_DIR}/kimera_pgmoConfig.cmake
  INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kimera_pgmo)
write_basic_package_version_file(
  kimera_pgmoConfigVersion.cmake
  VERSION ${PACKAGE_VERSION}
  COMPATIBILITY AnyNewerVersion)

install(FILES ${CMAKE_CURRENT_BINARY_DIR}/kimera_pgmoConfig.cmake
              ${CMAKE_CURRENT_BINARY_DIR}/kimera_pgmoConfigVersion.cmake
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/kimera_pgmo)
