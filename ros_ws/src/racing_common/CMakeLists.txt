cmake_minimum_required(VERSION 3.16)
project(racing_common)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter Development)
find_package(pybind11 REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(${PROJECT_NAME} SHARED src/track.cpp)
target_compile_features(${PROJECT_NAME} PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME} PRIVATE yaml-cpp)

pybind11_add_module(${PROJECT_NAME}_python MODULE python/bindings.cpp)
set_target_properties(${PROJECT_NAME}_python PROPERTIES
  OUTPUT_NAME ${PROJECT_NAME}
)
target_link_libraries(${PROJECT_NAME}_python PRIVATE ${PROJECT_NAME})
ament_get_python_install_dir(PYTHON_INSTALL_DIR)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS ${PROJECT_NAME}_python
  LIBRARY DESTINATION ${PYTHON_INSTALL_DIR}
)
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(track_test test/track_test.cpp)
  # The vendored gtest sources are third-party code and violate this repo's
  # clang-tidy rules. Keep authored targets in the compilation database.
  set_target_properties(gtest gtest_main PROPERTIES
    EXPORT_COMPILE_COMMANDS OFF
  )
  target_link_libraries(track_test ${PROJECT_NAME})
  target_compile_definitions(track_test PRIVATE
    RACING_COMMON_TEST_TRACK="${CMAKE_CURRENT_SOURCE_DIR}/../../../config/tracks/analytic_circle.yaml"
    RACING_COMMON_UNSUPPORTED_TEST_TRACK="${CMAKE_CURRENT_SOURCE_DIR}/test/unsupported_track.yaml"
  )
  set_tests_properties(track_test PROPERTIES LABELS "tier1")
endif()

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
