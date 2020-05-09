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

catkin_simple()

cs_add_library(${PROJECT_NAME}
src/CommonFunctions.cpp
src/DeformationGraph.cpp
src/CommonStructs.cpp
src/AlliezDesbrunCompression.cpp
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

cs_add_executable(example_from_ply
  src/example_from_ply.cpp
)
target_link_libraries(example_from_ply ${PROJECT_NAME})

cs_install()

cs_export()
