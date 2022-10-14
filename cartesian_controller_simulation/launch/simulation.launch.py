################################################################################
# Copyright 2022 FZI Research Center for Information Technology
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
################################################################################

# -----------------------------------------------------------------------------
# \file    simulation.launch.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2022/02/14
#
# -----------------------------------------------------------------------------


from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

import os
distro = os.environ['ROS_DISTRO']
if distro == 'humble' or distro == 'galactic':
    spawner = "spawner"
else:  # foxy
    spawner = "spawner.py"


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    # Build the URDF with command line xacro.
    # We also pass parameters for the system_interface here.
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("cartesian_controller_simulation"), "urdf", "robot.urdf.xacro"]
            ),
            " ",
            "mujoco_model:=",
            PathJoinSubstitution(
                [FindPackageShare("cartesian_controller_simulation"), "etc", "robot_mujoco.xml"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    robot_controllers = PathJoinSubstitution(
        [
            FindPackageShare("cartesian_controller_simulation"), "config", "controller_manager.yaml",
        ]
    )

    # The actual simulation is a ROS2-control system interface.
    # Start that with the usual ROS2 controller manager mechanisms.
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description, robot_controllers],
        #prefix="screen -d -m gdb -command=/home/scherzin/.ros/my_debug_log --ex run --args",
        output="both",
        remappings=[
            ('motion_control_handle/target_frame', 'target_frame'),
            ('cartesian_motion_controller/target_frame', 'target_frame'),
            ('cartesian_compliance_controller/target_frame', 'target_frame'),
            ('cartesian_force_controller/target_wrench', 'target_wrench'),
            ('cartesian_compliance_controller/target_wrench', 'target_wrench'),
            ('cartesian_force_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ('cartesian_compliance_controller/ft_sensor_wrench', 'ft_sensor_wrench'),
            ]
    )

    # Joint states
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
    )
    cartesian_compliance_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["cartesian_compliance_controller", "--stopped", "-c", "/controller_manager"],
    )
    cartesian_force_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["cartesian_force_controller", "--stopped", "-c", "/controller_manager"],
    )
    cartesian_motion_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["cartesian_motion_controller", "--stopped", "-c", "/controller_manager"],
    )
    motion_control_handle_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["motion_control_handle", "--stopped", "-c", "/controller_manager"],
    )
    joint_trajectory_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["joint_trajectory_controller", "--stopped", "-c", "/controller_manager"],
    )
    invalid_cartesian_compliance_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["invalid_cartesian_compliance_controller", "--stopped", "-c", "/controller_manager"],
    )
    invalid_cartesian_force_controller_spawner = Node(
        package="controller_manager",
        executable=spawner,
        arguments=["invalid_cartesian_force_controller", "--stopped", "-c", "/controller_manager"],
    )

    # TF tree
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description],
    )

    # Visualization
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("cartesian_controller_simulation"), "etc", "robot.rviz"]
    )
    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config]
    )

    # Nodes to start
    nodes = [
        control_node,
        joint_state_broadcaster_spawner,
        cartesian_compliance_controller_spawner,
        cartesian_force_controller_spawner,
        cartesian_motion_controller_spawner,
        motion_control_handle_spawner,
        joint_trajectory_controller_spawner,
        invalid_cartesian_compliance_controller_spawner,
        invalid_cartesian_force_controller_spawner,
        robot_state_publisher,
        rviz
    ]

    return LaunchDescription(declared_arguments + nodes)
