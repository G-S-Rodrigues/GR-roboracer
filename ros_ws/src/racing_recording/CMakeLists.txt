cmake_minimum_required(VERSION 3.16)
project(racing_recording)

find_package(ackermann_msgs REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(racing_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/event_recorder.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

add_executable(${PROJECT_NAME}_node src/node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  ackermann_msgs racing_interfaces rclcpp std_srvs
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
)
install(TARGETS ${PROJECT_NAME}_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(event_recorder_test test/event_recorder_test.cpp)
  set_target_properties(gtest gtest_main PROPERTIES
    EXPORT_COMPILE_COMMANDS OFF
  )
  target_link_libraries(event_recorder_test ${PROJECT_NAME})
  set_tests_properties(event_recorder_test PROPERTIES LABELS "tier1")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
