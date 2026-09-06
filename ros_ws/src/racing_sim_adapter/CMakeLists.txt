cmake_minimum_required(VERSION 3.16)
project(racing_sim_adapter)

find_package(ament_cmake REQUIRED)

install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_no_leak test/test_no_leak.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  set_tests_properties(test_no_leak PROPERTIES LABELS "tier2")

  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_adapter_contract.py
    TIMEOUT 30
    LABELS "tier2"
  )
endif()

ament_export_include_directories(include)
ament_package()
