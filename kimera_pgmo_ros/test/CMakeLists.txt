find_package(rostest REQUIRED)
find_package(roslib REQUIRED)

catkin_add_gtest(${PROJECT_NAME}-test pgmo_ros_unit_tests.cpp
                 test_mesh_conversion.cpp test_gtsam_conversions.cpp)
target_include_directories(${PROJECT_NAME}-test PUBLIC ${roslib_INCLUDE_DIRS})
target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME} ${roslib_LIBRARIES})

add_rostest_gtest(${PROJECT_NAME}-test_kimera_pgmo test_kimera_pgmo.test
                  test_kimera_pgmo.cpp)
target_include_directories(${PROJECT_NAME}-test_kimera_pgmo
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME}-test_kimera_pgmo ${PROJECT_NAME}
                      config_utilities::config_utilities)

add_rostest_gtest(${PROJECT_NAME}-test_kimera_dpgmo test_kimera_dpgmo.test
                  test_kimera_dpgmo.cpp)
target_include_directories(${PROJECT_NAME}-test_kimera_dpgmo
                           PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(${PROJECT_NAME}-test_kimera_dpgmo ${PROJECT_NAME}
                      config_utilities::config_utilities)
