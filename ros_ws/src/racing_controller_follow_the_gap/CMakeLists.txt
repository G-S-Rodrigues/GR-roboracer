cmake_minimum_required(VERSION 3.16)
project(racing_controller_follow_the_gap)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# ament_cmake rather than ament_python so the tier labels below exist
# (the same reason as racing_evaluation).
ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/racing_controller_follow_the_gap_node
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  foreach(tier1_test gap gap_porto)
    ament_add_pytest_test(test_${tier1_test} test/test_${tier1_test}.py
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}
    )
    set_tests_properties(test_${tier1_test} PROPERTIES LABELS "tier1")
  endforeach()

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_follow_the_gap_node.py
    TIMEOUT 30
    LABELS "tier2"
  )
endif()

ament_package()
