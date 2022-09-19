cmake_minimum_required(VERSION 3.5)
project(cartesian_motion_controller)

set(CMAKE_CXX_STANDARD 17)
set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cartesian_controller_base REQUIRED)

# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        rclcpp
        cartesian_controller_base
        Eigen3
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/cartesian_motion_controller.cpp
)

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  PRIVATE
    ${EIGEN3_INCLUDE_DIR}
)

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

pluginlib_export_plugin_description_file(controller_interface cartesian_motion_controller_plugin.xml)

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
