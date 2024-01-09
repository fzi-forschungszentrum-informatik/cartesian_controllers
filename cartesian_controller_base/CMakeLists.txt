cmake_minimum_required(VERSION 3.5)
project(cartesian_controller_base)

set(CMAKE_CXX_STANDARD 17)

set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

# Use CMake to pass the current ROS_DISTRO via variables into a preprocessor template.
# We then include this file and switch between the different APIs.
if($ENV{ROS_DISTRO} STREQUAL "iron")
        set(CARTESIAN_CONTROLLERS_IRON TRUE)
elseif($ENV{ROS_DISTRO} STREQUAL "humble")
        set(CARTESIAN_CONTROLLERS_HUMBLE TRUE)
elseif($ENV{ROS_DISTRO} STREQUAL "galactic")
        set(CARTESIAN_CONTROLLERS_GALACTIC TRUE)
elseif($ENV{ROS_DISTRO} STREQUAL "foxy")
        set(CARTESIAN_CONTROLLERS_FOXY TRUE)
else()
        message(WARNING "ROS2 version must be {iron|humble|galactic|foxy}")
endif()
configure_file(include/cartesian_controller_base/ROS2VersionConfig.h.in ROS2VersionConfig.h)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)
find_package(realtime_tools REQUIRED)


# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        controller_interface
        kdl_parser
        trajectory_msgs
        pluginlib
        urdf
        Eigen3
        realtime_tools
)

ament_export_dependencies(
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


#--------------------------------------------------------------------------------
# Libraries
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
  src/cartesian_controller_base.cpp
  src/SpatialPDController.cpp
  src/PDController.cpp
  src/IKSolver.cpp
)

# Manual includes for local directories and non-ament packages
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${CMAKE_BINARY_DIR}  # ROS2VersionConfig.h
)

ament_target_dependencies(${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


add_library(ik_solvers SHARED
  src/IKSolver.cpp
  src/ForwardDynamicsSolver.cpp
  src/JacobianTransposeSolver.cpp
  src/DampedLeastSquaresSolver.cpp
  src/SelectivelyDampedLeastSquaresSolver.cpp
)

target_include_directories(ik_solvers
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CMAKE_BINARY_DIR}  # ROS2VersionConfig.h
)

# Prevent pluginlib from using boost
target_compile_definitions(${PROJECT_NAME} PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

ament_target_dependencies(ik_solvers
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${THIS_PACKAGE_INCLUDE_DEPENDS}
)


#--------------------------------------------------------------------------------
# Install and export
#--------------------------------------------------------------------------------

pluginlib_export_plugin_description_file(cartesian_controller_base ik_solver_plugin.xml)

# Note: The workflow as described here
# https://docs.ros.org/en/foxy/How-To-Guides/Ament-CMake-Documentation.html#building-a-library
# clashes with colcon's --symlink-install option.
# See here: /opt/ros/foxy/share/ament_cmake_core/cmake/symlink_install/ament_cmake_symlink_install_targets.cmake
# which lists `EXPORT` and `INCLUDES` (among other install keywords) to trigger a normal resource install.

#ament_export_targets(my_targets_from_this_package HAS_LIBRARY_TARGET)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} ik_solvers
  #EXPORT my_targets_from_this_package
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  #INCLUDES DESTINATION include
)

install(
  FILES ${CMAKE_BINARY_DIR}/ROS2VersionConfig.h
  DESTINATION include/${PROJECT_NAME}
)

# Note: For the target based workflow, they seem to be superfluous.
# But since that doesn't work yet, I'll add them just in case.
# I took the joint_trajectory_controller as inspiration.
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_export_include_directories(
  include
)
ament_export_libraries(
  ${PROJECT_NAME} ik_solvers
)

ament_package()
