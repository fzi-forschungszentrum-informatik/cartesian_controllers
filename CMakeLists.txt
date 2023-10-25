cmake_minimum_required(VERSION 3.8)
project(rackki_learning)

# We need C++17
set(CMAKE_CXX_STANDARD 17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

# GLFW
# With PkgConfig is the official way: https://www.glfw.org/docs/3.0/build.html#build_link_cmake_pkgconfig
find_package(PkgConfig REQUIRED)
pkg_search_module(GLFW REQUIRED glfw3)

# GLEW
find_package(GLEW REQUIRED)

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
# Simulator
add_executable(${PROJECT_NAME}_simulator
        src/mujoco_simulator.cpp
        src/simulator_node.cpp
        )

target_include_directories(${PROJECT_NAME}_simulator PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${MUJOCO_INCLUDE_DIRS}
        ${GLFW_INCLUDE_DIRS}
        )

ament_target_dependencies(${PROJECT_NAME}_simulator
        rclcpp
        geometry_msgs
        )

target_link_libraries(${PROJECT_NAME}_simulator
        ${MUJOCO_LIBRARIES}
        ${GLFW_LIBRARIES}
        GLEW::GLEW
        )


#--------------------------------------------------------------------------------
# Install
#--------------------------------------------------------------------------------
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch meshes
  DESTINATION share/${PROJECT_NAME}
)

install(
  TARGETS ${PROJECT_NAME}_simulator
  DESTINATION lib/${PROJECT_NAME})


if(BUILD_TESTING)
endif()

ament_package()
