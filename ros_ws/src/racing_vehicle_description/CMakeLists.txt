cmake_minimum_required(VERSION 3.16)
project(racing_vehicle_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY urdf/ DESTINATION share/${PROJECT_NAME}/urdf)
install(DIRECTORY rviz/ DESTINATION share/${PROJECT_NAME}/rviz)
install(DIRECTORY meshes/ DESTINATION share/${PROJECT_NAME}/meshes)
install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_description test/test_description.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  set_tests_properties(test_description PROPERTIES LABELS "tier1")
  ament_add_pytest_test(test_urdf_matches_vehicle
    test/test_urdf_matches_vehicle.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  set_tests_properties(test_urdf_matches_vehicle PROPERTIES LABELS "tier1")
  ament_add_pytest_test(test_autodrive_description
    test/test_autodrive_description.py
    APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_SOURCE_DIR}/test
  )
  set_tests_properties(test_autodrive_description PROPERTIES LABELS "tier1")
  find_package(launch_testing_ament_cmake REQUIRED)
  add_launch_test(test/test_autodrive_tf_launch.py TIMEOUT 30
    LABELS "tier2"
  )
endif()

ament_package()
