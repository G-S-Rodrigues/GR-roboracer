cmake_minimum_required(VERSION 3.16)
project(racing_safety_supervisor)

find_package(ackermann_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(racing_common REQUIRED)
find_package(racing_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/supervisor.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PUBLIC racing_common::racing_common)

add_executable(${PROJECT_NAME}_node src/node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  ackermann_msgs nav_msgs racing_interfaces rclcpp std_srvs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY include/ DESTINATION include)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(supervisor_test test/supervisor_test.cpp)
  set_target_properties(gtest gtest_main PROPERTIES
    EXPORT_COMPILE_COMMANDS OFF
  )
  target_compile_features(supervisor_test PUBLIC cxx_std_17)
  target_include_directories(supervisor_test PRIVATE include)
  target_link_libraries(supervisor_test ${PROJECT_NAME})
  target_compile_definitions(supervisor_test PRIVATE
    RACING_SAFETY_TEST_TRACK="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/tracks/analytic_circle.yaml"
  )
  set_tests_properties(supervisor_test PROPERTIES LABELS "tier1")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_dependencies(racing_common)
ament_package()
