cmake_minimum_required(VERSION 3.16)
project(racing_controller_baseline)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(racing_interfaces REQUIRED)
find_package(rclcpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/pure_pursuit.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

pybind11_add_module(${PROJECT_NAME}_python MODULE python/bindings.cpp)
set_target_properties(${PROJECT_NAME}_python PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_python PRIVATE ${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

add_executable(${PROJECT_NAME}_node src/node.cpp)
target_compile_features(${PROJECT_NAME}_node PUBLIC cxx_std_17)
target_link_libraries(${PROJECT_NAME}_node ${PROJECT_NAME})
ament_target_dependencies(${PROJECT_NAME}_node
  ackermann_msgs nav_msgs racing_interfaces rclcpp
)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_python
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
)
install(TARGETS ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(pure_pursuit_test test/pure_pursuit_test.cpp)
  set_target_properties(gtest gtest_main PROPERTIES
    EXPORT_COMPILE_COMMANDS OFF
  )
  target_link_libraries(pure_pursuit_test ${PROJECT_NAME})
  set_tests_properties(pure_pursuit_test PROPERTIES LABELS "tier1")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
