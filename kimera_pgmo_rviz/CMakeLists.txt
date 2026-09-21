cmake_minimum_required(VERSION 3.16)
project(kimera_pgmo_rviz)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(kimera_pgmo_ros REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_common REQUIRED)
find_package(yaml-cpp REQUIRED)

set(CMAKE_AUTOMOC ON)

add_library(
  kimera_pgmo_rviz_coloring
  SHARED
  src/mesh_coloring_plugin.cpp
  src/mesh_coloring_property.cpp
  include/kimera_pgmo_rviz/mesh_coloring_plugin.h
  include/kimera_pgmo_rviz/mesh_coloring_property.h)
target_include_directories(
  kimera_pgmo_rviz_coloring
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
ament_target_dependencies(kimera_pgmo_rviz_coloring PUBLIC kimera_pgmo_ros pluginlib
                          rviz_common)
target_link_options(kimera_pgmo_rviz_coloring PRIVATE -Wl,--no-undefined)

# Keep registrations out of libraries linked by the display so pluginlib owns
# their initial load and associates factories with the coloring loader.
add_library(kimera_pgmo_rviz_coloring_plugins SHARED src/mesh_coloring_plugins.cpp)
target_link_libraries(kimera_pgmo_rviz_coloring_plugins
                      PRIVATE kimera_pgmo_rviz_coloring yaml-cpp::yaml-cpp)
target_link_options(kimera_pgmo_rviz_coloring_plugins PRIVATE -Wl,--no-undefined)
pluginlib_export_plugin_description_file(kimera_pgmo_rviz mesh_coloring_plugins.xml)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/mesh_delta_display.cpp
  src/mesh_display.cpp
  src/mesh_visual.cpp
  src/mesh_geometry.cpp
  src/mesh_properties.cpp
  src/tf_event_buffer.cpp
  src/visibility_field.cpp
  src/visuals_map.cpp
  include/kimera_pgmo_rviz/mesh_delta_display.h
  include/kimera_pgmo_rviz/mesh_display.h
  include/kimera_pgmo_rviz/mesh_properties.h
  include/kimera_pgmo_rviz/visibility_field.h)
target_include_directories(
  ${PROJECT_NAME}
  PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
         "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")
target_link_libraries(${PROJECT_NAME} PUBLIC kimera_pgmo_rviz_coloring)
target_link_options(${PROJECT_NAME} PRIVATE -Wl,--no-undefined)
ament_target_dependencies(${PROJECT_NAME} PUBLIC kimera_pgmo_ros pluginlib
                          rviz_common)

install(
  TARGETS ${PROJECT_NAME} kimera_pgmo_rviz_coloring
  EXPORT ${PROJECT_NAME}-targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(TARGETS kimera_pgmo_rviz_coloring_plugins LIBRARY DESTINATION lib)
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION include/${PROJECT_NAME}/)
install(FILES rviz_common_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY icons/ DESTINATION share/${PROJECT_NAME}/icons)

pluginlib_export_plugin_description_file(rviz_common rviz_common_plugins.xml)
ament_export_targets(${PROJECT_NAME}-targets)
ament_export_dependencies(kimera_pgmo_ros pluginlib rviz_common)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_mesh_geometry test/test_mesh_geometry.cpp src/mesh_geometry.cpp)
  target_include_directories(test_mesh_geometry PRIVATE include)
  ament_target_dependencies(test_mesh_geometry kimera_pgmo_ros)

  ament_add_gtest(test_mesh_coloring_property test/property_test_main.cpp
                  test/test_mesh_coloring_property.cpp)
  target_link_libraries(test_mesh_coloring_property ${PROJECT_NAME})

  # Load the display through pluginlib, as RViz does, rather than linking it.
  ament_add_gtest(test_mesh_coloring_loading test/property_test_main.cpp
                  test/test_mesh_coloring_loading.cpp)
  ament_target_dependencies(test_mesh_coloring_loading pluginlib rviz_common)
endif()

ament_package()
