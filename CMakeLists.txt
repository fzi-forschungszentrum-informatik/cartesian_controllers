cmake_minimum_required(VERSION 3.8)
project(rackki_learning)

# We need C++17
set(CMAKE_CXX_STANDARD 17)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(rackki_interfaces REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)

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

# Python
find_package(ament_cmake_python REQUIRED)

# Tensorflow's C++ API
# We use ika-rwth-aachen's prebuild .deb package.
# Install that according to their install instructions.
set(TENSORFLOW_INCLUDE_DIRS /usr/local/include/tensorflow)
set(TENSORFLOW_LIBRARIES
        /usr/local/lib/libtensorflow_cc.so
        /usr/local/lib/libtensorflow_framework.so
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
target_compile_options(${PROJECT_NAME}_simulator PUBLIC
        -Wall -Wextra -Wpedantic
        )
target_compile_definitions(${PROJECT_NAME}_simulator PRIVATE
        MUJOCO_PLUGIN_DIR="${MUJOCO_DIR}/bin/mujoco_plugin"
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
        std_srvs
        )
target_link_libraries(${PROJECT_NAME}_simulator
        ${MUJOCO_LIBRARIES}
        ${GLFW_LIBRARIES}
        GLEW::GLEW
        )

# Skill controller
add_library(skill_controller SHARED
        src/skill_controller.cpp
        )
target_include_directories(skill_controller PUBLIC
        $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${TENSORFLOW_INCLUDE_DIRS}
        )
ament_target_dependencies(skill_controller
        rclcpp
        controller_interface
        kdl_parser
        urdf
        rackki_interfaces
        tf2
        tf2_ros
        )
target_link_libraries(skill_controller
        ${TENSORFLOW_LIBRARIES}
        yaml-cpp
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
  TARGETS
  ${PROJECT_NAME}_simulator
  skill_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/serve_model.py
  scripts/teach_device.py
  scripts/train_model.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})
pluginlib_export_plugin_description_file(controller_interface skill_controller_plugin.xml)

#--------------------------------------------------------------------------------
# Test
#--------------------------------------------------------------------------------
if(BUILD_TESTING)

  # ML components in Python
  # See: https://docs.ros.org/en/rolling/How-To-Guides/Ament-CMake-Python-Documentation.html
  find_package(ament_cmake_pytest REQUIRED)
  set(_pytest_tests
    test/python/test_dataset.py
    test/python/test_model.py
  )

  foreach(_test_path ${_pytest_tests})
    get_filename_component(_test_name ${_test_path} NAME_WE)
    ament_add_pytest_test(${_test_name} ${_test_path}
      APPEND_ENV PYTHONPATH=${CMAKE_CURRENT_BINARY_DIR}
      TIMEOUT 60
      WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    )
  endforeach()

  # Prediction in C++
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(${PROJECT_NAME}_test_prediction
    test/cpp/test_prediction.cpp
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}/test/models
  )
  target_include_directories(${PROJECT_NAME}_test_prediction PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${TENSORFLOW_INCLUDE_DIRS}
  )
  ament_target_dependencies(${PROJECT_NAME}_test_prediction
  )
  target_link_libraries(${PROJECT_NAME}_test_prediction
    ${TENSORFLOW_LIBRARIES}
  )

  # ROS2 controller integration
  install(
    DIRECTORY test/config test/urdf
    DESTINATION share/${PROJECT_NAME}
  )
  find_package(launch_testing_ament_cmake)
  add_launch_test(test/launch/test_skill_controller.py)

endif()

ament_package()
