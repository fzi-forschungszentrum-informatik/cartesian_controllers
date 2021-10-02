cmake_minimum_required(VERSION 3.5)
project(cartesian_controller_base)

set(CMAKE_CXX_STANDARD 17)

set(ADDITIONAL_COMPILE_OPTIONS -Wall -Wextra -Wpedantic -Wno-unused-parameter)
add_compile_options(${ADDITIONAL_COMPILE_OPTIONS})

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
add_definitions(-DEIGEN_MPL2_ONLY)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(controller_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(urdf REQUIRED)


# Convenience variable for dependencies
set(THIS_PACKAGE_INCLUDE_DEPENDS
        controller_interface
        kdl_parser
        trajectory_msgs
        pluginlib
        urdf
        Eigen3
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
ament_export_targets(my_targets_from_this_package HAS_LIBRARY_TARGET)

pluginlib_export_plugin_description_file(cartesian_controller_base ik_solver_plugin.xml)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME} ik_solvers
  EXPORT my_targets_from_this_package
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)


ament_package()
