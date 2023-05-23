cmake_minimum_required(VERSION 3.10)
project(cartesian_controller_tests)

find_package(ament_cmake REQUIRED)


if(BUILD_TESTING)
  find_package(cartesian_controller_base REQUIRED)
  find_package(cartesian_motion_controller REQUIRED)
  find_package(cartesian_force_controller REQUIRED)
  find_package(cartesian_compliance_controller REQUIRED)
  find_package(cartesian_controller_handles REQUIRED)
  find_package(cartesian_controller_simulation REQUIRED)
  find_package(controller_manager_msgs REQUIRED)
  find_package(launch_testing_ament_cmake)
  add_launch_test(integration_tests/integration_tests.py)
endif()

ament_package()
