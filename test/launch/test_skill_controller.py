#!/usr/bin/env python3
import unittest
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import TimerAction
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node
from launch.substitutions import (
    Command,
    FindExecutable,
    PathJoinSubstitution,
)
import rclpy
from rclpy.node import Node as PythonNode
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from rackki_interfaces.srv import SetTarget
import os

distro = os.environ["ROS_DISTRO"]


def generate_launch_description():
    robot_description_content = Command(
        [
            FindExecutable(name="xacro"),
            " ",
            PathJoinSubstitution(
                [FindPackageShare("rackki_learning"), "urdf", "robot.urdf.xacro"]
            ),
        ]
    )
    robot_description = {"robot_description": robot_description_content}

    controller_manager_config = PathJoinSubstitution(
        [FindPackageShare("rackki_learning"), "config", "controller_manager.yaml"]
    )
    control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        parameters=[robot_description, controller_manager_config],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )

    def controller_spawner(name, *args):
        return Node(
            package="controller_manager",
            executable="spawner",
            output="screen",
            arguments=[name] + [a for a in args],
        )

    active_list = [
        "joint_state_broadcaster",
    ]
    active_spawners = [controller_spawner(controller) for controller in active_list]

    inactive_list = [
        "skill_controller",
    ]
    inactive_flag = (
        "--inactive" if distro in ["rolling", "iron", "humble"] else "--stopped"
    )
    inactive_spawners = [
        controller_spawner(controller, inactive_flag) for controller in inactive_list
    ]

    nodes = [control_node, robot_state_publisher] + active_spawners + inactive_spawners
    return LaunchDescription(nodes)


def generate_test_description():
    setup = generate_launch_description()
    until_ready = 3.0  # sec
    return LaunchDescription(
        [
            setup,
            TimerAction(
                period=until_ready, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    )


class SkillControllerTests(unittest.TestCase):
    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = PythonNode("test_skill_controller")
        cls.setup_interfaces(cls)

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_1_lifecycle(self):
        controller = "skill_controller"
        self.assertTrue(self.check_state(controller, "inactive"))
        for _ in range(3):
            self.assertTrue(self.update_target("tool0"))
            self.start_controller(controller)
            self.assertTrue(self.check_state(controller, "active"))
            self.stop_controller(controller)
            self.assertTrue(self.check_state(controller, "inactive"))

    def setup_interfaces(self):
        timeout = rclpy.time.Duration(seconds=5)
        self.list_controllers = self.node.create_client(
            ListControllers, "/controller_manager/list_controllers"
        )
        if not self.list_controllers.wait_for_service(timeout.nanoseconds / 1e9):
            self.fail("Service list_controllers not available")

        self.switch_controller = self.node.create_client(
            SwitchController, "/controller_manager/switch_controller"
        )
        if not self.switch_controller.wait_for_service(timeout.nanoseconds / 1e9):
            self.fail("Service switch_controllers not available")

        self.set_target = self.node.create_client(
            SetTarget, "/skill_controller/set_target"
        )
        if not self.set_target.wait_for_service(timeout.nanoseconds / 1e9):
            self.fail("Service set_target not available")

    def check_state(self, controller, state):
        req = ListControllers.Request()
        future = self.list_controllers.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        for entry in future.result().controller:
            if entry.name == controller:
                return True if entry.state == state else False
        return False

    def start_controller(self, controller):
        req = SwitchController.Request()
        if distro in ["rolling", "humble", "iron"]:
            req.activate_controllers = [controller]
        else:
            req.start_controllers = [controller]
        self.perform_switch(req)

    def stop_controller(self, controller):
        req = SwitchController.Request()
        if distro in ["rolling", "humble", "iron"]:
            req.deactivate_controllers = [controller]
        else:
            req.stop_controllers = [controller]
        self.perform_switch(req)

    def perform_switch(self, req):
        req.strictness = req.BEST_EFFORT
        future = self.switch_controller.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)

    def update_target(self, target):
        req = SetTarget.Request()
        req.tf_name = target
        future = self.set_target.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        return future.result().success
