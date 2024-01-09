#!/usr/bin/env python3
import unittest

import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import os
import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped

distro = os.environ["ROS_DISTRO"]


def generate_test_description():
    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [
                FindPackageShare("cartesian_controller_simulation"),
                "launch",
                "simulation.launch.py",
            ]
        )
    )
    until_ready = 10.0  # sec
    return LaunchDescription(
        [
            setup,
            TimerAction(
                period=until_ready, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    )


class IntegrationTest(unittest.TestCase):
    """Integration tests for the cartesian controllers in a simulation setting

    We check if each controller successfully performs the life cycle of
    `initialized` - `active` - `update` - `inactive`, and whether it behaves
    correctly in selected use cases.
    """

    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_startup")
        cls.setup_interfaces(cls)

        cls.our_controllers = [
            "cartesian_motion_controller",
            "cartesian_force_controller",
            "cartesian_compliance_controller",
            "motion_control_handle",
        ]
        cls.invalid_controllers = [
            "invalid_cartesian_force_controller",
            "invalid_cartesian_compliance_controller",
        ]

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setup_interfaces(self):
        """Setup interfaces for ROS2 services and topics"""

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

        self.target_pose_pub = self.node.create_publisher(
            PoseStamped, "target_frame", 3
        )
        self.target_wrench_pub = self.node.create_publisher(
            WrenchStamped, "target_wrench", 3
        )
        self.ft_sensor_wrench_pub = self.node.create_publisher(
            WrenchStamped, "ft_sensor_wrench", 3
        )

    def test_controller_initialization(self):
        """Test whether every controller got initialized correctly

        We check if the list of all controllers currently managed by the
        controller manager contains our controllers and if they have `state:
        inactive`.
        """
        for name in self.our_controllers:
            self.assertTrue(
                self.check_state(name, "inactive"), f"{name} is initialized correctly"
            )

    def test_invalid_controller_initialization(self):
        """Test whether the invalid controllers' initialization fails as expected

        We check if the list of all controllers currently managed by the
        controller manager contains our controllers and if they have the
        expected state.
        """
        if os.environ["ROS_DISTRO"] == "humble" or os.environ["ROS_DISTRO"] == "iron":
            expected_state = "unconfigured"
        else:  # galactic, foxy
            expected_state = "finalized"
        for name in self.invalid_controllers:
            self.assertTrue(
                self.check_state(name, expected_state),
                f"{name} initializes although it should not.",
            )

    def test_controller_switches(self):
        """Test whether every controller starts, runs, and stops correctly

        We start each of our controllers individually and check if its state is
        `active`.  We then switch it off and check whether its state is
        `inactive`.
        """
        for name in self.our_controllers:
            self.start_controller(name)
            self.assertTrue(
                self.check_state(name, "active"),
                "{} is starting correctly".format(name),
            )
            time.sleep(3)  # process some update() cycles
            self.stop_controller(name)
            self.assertTrue(
                self.check_state(name, "inactive"),
                "{} is stopping correctly".format(name),
            )

    def test_inputs_with_nans(self):
        """Test whether every controller survives inputs with NaNs

        We publish invalid values to all input topics.
        The controllers' callbacks store them even when in inactive state after startup.
        We then check if every controller can be activated normally.
        """
        # Target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = float("nan")
        target_pose.pose.position.y = float("nan")
        target_pose.pose.position.z = float("nan")
        target_pose.pose.orientation.x = float("nan")
        target_pose.pose.orientation.y = float("nan")
        target_pose.pose.orientation.z = float("nan")
        target_pose.pose.orientation.w = float("nan")
        self.target_pose_pub.publish(target_pose)

        # Force-torque sensor
        ft_sensor_wrench = WrenchStamped()
        ft_sensor_wrench.wrench.force.x = float("nan")
        ft_sensor_wrench.wrench.force.y = float("nan")
        ft_sensor_wrench.wrench.force.z = float("nan")
        ft_sensor_wrench.wrench.torque.x = float("nan")
        ft_sensor_wrench.wrench.torque.y = float("nan")
        ft_sensor_wrench.wrench.torque.z = float("nan")
        self.ft_sensor_wrench_pub.publish(ft_sensor_wrench)

        # Target wrench
        target_wrench = WrenchStamped()
        target_wrench.wrench.force.x = float("nan")
        target_wrench.wrench.force.y = float("nan")
        target_wrench.wrench.force.z = float("nan")
        target_wrench.wrench.torque.x = float("nan")
        target_wrench.wrench.torque.y = float("nan")
        target_wrench.wrench.torque.z = float("nan")
        self.target_wrench_pub.publish(target_wrench)

        time.sleep(1)  # give the controllers some time to process

        for name in self.our_controllers:
            self.start_controller(name)
            time.sleep(3)  # process some update() cycles
            self.assertTrue(
                self.check_state(name, "active"), f"{name} survives inputs with NaNs"
            )
            self.stop_controller(name)

    def check_state(self, controller, state):
        """Check the controller's state

        Return True if the controller's state is `state`, else False.
        Return False if the controller is not listed.
        """
        req = ListControllers.Request()
        future = self.list_controllers.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
        for entry in future.result().controller:
            if entry.name == controller:
                return True if entry.state == state else False
        return False

    def start_controller(self, controller):
        """Start the given controller"""
        req = SwitchController.Request()
        if distro in ["humble", "iron"]:
            req.activate_controllers = [controller]
        else:
            req.start_controllers = [controller]
        self.perform_switch(req)

    def stop_controller(self, controller):
        """Stop the given controller"""
        req = SwitchController.Request()
        if distro in ["humble", "iron"]:
            req.deactivate_controllers = [controller]
        else:
            req.stop_controllers = [controller]
        self.perform_switch(req)

    def perform_switch(self, req):
        """Trigger the controller switch with the given request"""
        req.strictness = req.BEST_EFFORT
        future = self.switch_controller.call_async(req)
        rclpy.spin_until_future_complete(self.node, future)
