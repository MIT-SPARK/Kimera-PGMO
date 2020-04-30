cmake_minimum_required(VERSION 2.8.3)
project(mesher_mapper)

find_package(catkin_simple REQUIRED)

### System Dependencies
find_package(Boost REQUIRED COMPONENTS
  serialization system filesystem thread date_time regex timer chrono)
if(NOT TARGET Boost::boost)
  add_library(Boost::boost INTERFACE IMPORTED)
  set_target_properties(Boost::boost PROPERTIES
  INTERFACE_LINK_LIBRARIES "${Boost_LIBRARIES}"
  INTERFACE_INCLUDE_DIRECTORIES "${Boost_INCLUDE_DIRS}")
endif()

catkin_simple()

cs_add_library(${PROJECT_NAME}
#
)
target_link_libraries(${PROJECT_NAME} 
  ${catkin_LIBRARIES}
  ${Boost_LIBRARIES}
  gtsam
)

# cs_add_executable(${PROJECT_NAME}_node
  # src/KimeraVioRosNode.cpp
# )
# target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})

# catkin_add_gtest(testKimeraVioRos
  # test/testKimeraVioRos.cpp
# )
# target_link_libraries(testKimeraVioRos ${PROJECT_NAME})

cs_install()

cs_export()
