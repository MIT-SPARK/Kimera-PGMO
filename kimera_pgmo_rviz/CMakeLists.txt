cmake_minimum_required(VERSION 3.16)
project(kimera_pgmo_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(kimera_pgmo_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mesh_display.cpp
  src/mesh_visual.cpp
  src/tf_event_buffer.cpp
  src/visibility_field.cpp
  src/visuals_map.cpp
  include/kimera_pgmo_rviz/mesh_display.h
  include/kimera_pgmo_rviz/visibility_field.h)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
ament_target_dependencies(${PROJECT_NAME} PUBLIC kimera_pgmo_msgs pluginlib
                          rviz_common)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME}/)
install(FILES rviz_common_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY icons/ DESTINATION share/${PROJECT_NAME}/icons)

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)
ament_export_targets(${PROJECT_NAME}-targets)
ament_package()
