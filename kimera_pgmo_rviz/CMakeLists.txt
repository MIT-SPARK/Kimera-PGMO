cmake_minimum_required(VERSION 3.12)
project(kimera_pgmo_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_compile_options(-Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS kimera_pgmo_msgs roscpp rviz)

# from
# https://github.com/ros-visualization/visualization_tutorials/blob/noetic-devel/rviz_plugin_tutorials/CMakeLists.txt
set(CMAKE_AUTOMOC ON)
find_package(Qt5 ${rviz_QT_VERSION} EXACT REQUIRED Core Widgets)
set(QT_LIBRARIES Qt5::Widgets)

catkin_package(
  CATKIN_DEPENDS
  kimera_pgmo_msgs
  roscpp
  rviz
  DEPENDS
  INCLUDE_DIRS
  include
  LIBRARIES)

add_library(${PROJECT_NAME} 
  src/mesh_display.cpp 
  src/mesh_visual.cpp
  src/tf_event_buffer.cpp
  src/visibility_field.cpp
  src/visuals_map.cpp
  include/kimera_pgmo_rviz/mesh_display.h
  include/kimera_pgmo_rviz/visibility_field.h
)
target_include_directories(${PROJECT_NAME} PUBLIC include
                                                  ${catkin_INCLUDE_DIRS})
target_compile_options(${PROJECT_NAME} PRIVATE -Wno-register
                                               -Wno-deprecated-copy)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES}
                                             ${QT_LIBRARIES})

install(
  TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
install(DIRECTORY include/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(FILES plugins.xml DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
install(DIRECTORY icons/ DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/icons)

