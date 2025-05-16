cmake_minimum_required(VERSION 3.5)
project(joint_to_cartesian_controller)

set(CMAKE_CXX_STANDARD 17)

set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

find_package(ament_cmake REQUIRED)
find_package(cartesian_controller_base REQUIRED) # For ROS2 version handling
find_package(controller_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(urdf REQUIRED)

# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        cartesian_controller_base
        controller_interface
        controller_manager
        geometry_msgs
        kdl_parser
        pluginlib
        rclcpp
        sensor_msgs
        urdf
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------

add_library(${PROJECT_NAME} SHARED
  src/joint_to_cartesian_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------


pluginlib_export_plugin_description_file(controller_interface joint_to_cartesian_controller_plugin.xml)

# Note: The workflow as described here https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library
# does not work for me.
# I'm not sure what's the problem right now, but I need to out-comment the
# lines below to achieve a correct symlink-install.

#ament_export_targets(my_targets_from_this_package HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  #EXPORT my_targets_from_this_package
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

# Note: For the target based workflow, they seem to be superfluous.
# But since that doesn't work yet, I'll add them just in case.
# I took the joint_trajectory_controller as inspiration.
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME}
)

ament_package()
