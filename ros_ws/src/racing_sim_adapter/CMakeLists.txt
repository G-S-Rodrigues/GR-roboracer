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
  ament_add_pytest_test(test_capabilities test/test_capabilities.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  set_tests_properties(test_capabilities PROPERTIES LABELS "tier1")

  find_package(launch_testing_ament_cmake REQUIRED)
  # An ENV written after LABELS is swallowed into LABELS (seen in
  # CTestTestfile.cmake, and every _autodrive target then ran the gym); one
  # written before it would reach ament_add_test. The backend is set as a
  # test property instead (ADAPT-1050 holds one per registration).
  # RACING_SIM_BACKEND is set on every registration so an exported variable
  # cannot turn a gym target into an autodrive run.
  add_launch_test(test/test_adapter_contract.py
    TIMEOUT 30
    LABELS "tier2"
  )
  set_property(TEST test_test_adapter_contract.py APPEND PROPERTY ENVIRONMENT
    RACING_SIM_BACKEND=f1tenth_gym_jax)
  add_launch_test(test/test_adapter_start_held.py
    TIMEOUT 30
    LABELS "tier2"
  )
  set_property(TEST test_test_adapter_start_held.py APPEND PROPERTY ENVIRONMENT
    RACING_SIM_BACKEND=f1tenth_gym_jax)
  add_launch_test(test/test_adapter_laser_origin.py
    TIMEOUT 30
    LABELS "tier2"
  )
  set_property(TEST test_test_adapter_laser_origin.py APPEND PROPERTY ENVIRONMENT
    RACING_SIM_BACKEND=f1tenth_gym_jax)
  add_launch_test(test/test_adapter_time_scale.py
    TIMEOUT 30
    LABELS "tier2"
  )
  set_property(TEST test_test_adapter_time_scale.py APPEND PROPERTY ENVIRONMENT
    RACING_SIM_BACKEND=f1tenth_gym_jax)
  # The same four files against the AutoDRIVE server node fed by the replay
  # (D6), after all the gym registrations.
  add_launch_test(test/test_adapter_contract.py
    TARGET test_adapter_contract_autodrive
    TIMEOUT 30
    LABELS "tier2"
  )
  set_property(TEST test_adapter_contract_autodrive APPEND PROPERTY ENVIRONMENT
    RACING_SIM_BACKEND=autodrive)
  add_launch_test(test/test_adapter_start_held.py
    TARGET test_adapter_start_held_autodrive
    TIMEOUT 30
    LABELS "tier2"
  )
  set_property(TEST test_adapter_start_held_autodrive APPEND PROPERTY ENVIRONMENT
    RACING_SIM_BACKEND=autodrive)
  add_launch_test(test/test_adapter_laser_origin.py
    TARGET test_adapter_laser_origin_autodrive
    TIMEOUT 30
    LABELS "tier2"
  )
  set_property(TEST test_adapter_laser_origin_autodrive APPEND PROPERTY ENVIRONMENT
    RACING_SIM_BACKEND=autodrive)
  add_launch_test(test/test_adapter_time_scale.py
    TARGET test_adapter_time_scale_autodrive
    TIMEOUT 30
    LABELS "tier2"
  )
  set_property(TEST test_adapter_time_scale_autodrive APPEND PROPERTY ENVIRONMENT
    RACING_SIM_BACKEND=autodrive)
endif()

ament_export_include_directories(include)
ament_package()
