cmake_minimum_required(VERSION 3.5)
project(cartesian_controller_utilities)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

install(PROGRAMS
  scripts/buttons.py
  scripts/converter.py
  scripts/pose.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY etc launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
