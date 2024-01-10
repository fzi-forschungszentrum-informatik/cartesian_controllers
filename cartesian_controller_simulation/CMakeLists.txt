cmake_minimum_required(VERSION 3.10)
project(cartesian_controller_simulation)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# We need C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(cartesian_controller_base REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

# Mujoco comes as a pre-built library package.
# Extract this package somewhere and point CMake towards it during the build process.
if(NOT DEFINED MUJOCO_DIR)
        set(MUJOCO_DIR "$ENV{HOME}/mujoco-3.0.0") # CI setting
        message(WARNING "MUJOCO_DIR not specified. Using default ${MUJOCO_DIR}")
endif()
set(MUJOCO_INCLUDE_DIRS ${MUJOCO_DIR}/include)
set(MUJOCO_LIBRARIES
        ${MUJOCO_DIR}/lib/libmujoco.so
)

# --------------------------------------------------------------------------------
# Make sure that library relocation works for both build and install.
# See here: https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling
# --------------------------------------------------------------------------------

set(CMAKE_SKIP_BUILD_RPATH FALSE)
set(CMAKE_BUILD_WITH_INSTALL_RPATH FALSE)
set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# The RPATH to use when installing, but only if it's not a system directory
list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
if("${isSystemDir}" STREQUAL "-1")
    set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
endif()

#--------------------------------------------------------------------------------
# Build
#--------------------------------------------------------------------------------
add_library(${PROJECT_NAME} SHARED
        src/system_interface.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
        cartesian_controller_base
        hardware_interface
        pluginlib
        rclcpp
)

target_link_libraries(${PROJECT_NAME}
        mujoco_simulator
)

pluginlib_export_plugin_description_file(hardware_interface cartesian_controller_simulation.xml)

#--------------------------------------------------------------------------------

add_library(mujoco_simulator SHARED
        src/mujoco_simulator.cpp
)

target_include_directories(mujoco_simulator PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${MUJOCO_INCLUDE_DIRS}
)

# X11 OpenGL: link with libmujoco210.so, libglew.so, libGL.so
target_link_libraries(mujoco_simulator PUBLIC
        ${MUJOCO_LIBRARIES}
)



#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config etc launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME} mujoco_simulator
  DESTINATION lib
)


#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
#if(BUILD_TESTING)
#  find_package(ament_lint_auto REQUIRED)
#  # the following line skips the linter which checks for copyrights
#  # uncomment the line when a copyright and license is not present in all source files
#  #set(ament_cmake_copyright_FOUND TRUE)
#  # the following line skips cpplint (only works in a git repo)
#  # uncomment the line when this package is not in a git repo
#  #set(ament_cmake_cpplint_FOUND TRUE)
#  ament_lint_auto_find_test_dependencies()
#endif()

ament_package()
