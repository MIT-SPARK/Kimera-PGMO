cmake_minimum_required(VERSION 3.5)
project(mesher_mapper)

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

find_package(PCL REQUIRED COMPONENTS common visualization octree io features filters kdtree surface)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_simple()

cs_add_library(${PROJECT_NAME}
src/CommonFunctions.cpp
src/DeformationGraph.cpp
src/CommonStructs.cpp
src/AlliezDesbrunCompression.cpp
src/OctreeCompression.cpp
src/tinyply.cpp
src/MesherMapper.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  ${PCL_LIBRARIES}
  gtsam
)

cs_add_executable(alliez_desbrun
  src/alliez_desbrun.cpp
)
target_link_libraries(alliez_desbrun ${PROJECT_NAME})

cs_add_executable(load_mesh
  src/load_mesh.cpp
)
target_link_libraries(load_mesh ${PROJECT_NAME})

# Unit tests 
set(TEST_DATA_PATH "${CMAKE_CURRENT_SOURCE_DIR}/test/data")
configure_file(test/test_config.h.in test/test_config.h)
include_directories(${CMAKE_CURRENT_BINARY_DIR}/test)
catkin_add_gtest(test_mesher_mapper 
  test/testCommonFunctions.cpp
  test/testDeformationEdgeFactor.cpp
  test/test_mesher_mapper.cpp)
target_link_libraries(test_mesher_mapper ${PROJECT_NAME} -ltbb)

cs_install()

cs_export()


