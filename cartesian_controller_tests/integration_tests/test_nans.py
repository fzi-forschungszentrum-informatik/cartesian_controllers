#!/usr/bin/env python3
import unittest

import launch
import launch.actions
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

import time
import rclpy
from rclpy.node import Node
from controller_manager_msgs.srv import ListControllers
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import WrenchStamped


def generate_test_description():

    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("cartesian_controller_simulation"), "launch", "simulation.launch.py"]
        )
    )
    until_ready = 10.0 # sec
    return LaunchDescription([setup, TimerAction(period=until_ready, actions=[launch_testing.actions.ReadyToTest()])])


class IntegrationTest(unittest.TestCase):
    """ An integration test for rejecting NaNs in input topics

    We check if NaNs are handled correctly.
    """
    def __init__(self, *args):
        super().__init__(*args)

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node("test_nans")
        cls.setup_interfaces(cls)

        cls.our_controllers = [
            'cartesian_motion_controller',
            'cartesian_force_controller',
            'cartesian_compliance_controller',
        ]


    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def setup_interfaces(self):
        """ Setup interfaces for ROS2 services and topics """

        timeout = rclpy.time.Duration(seconds=5)
        self.list_controllers = self.node.create_client(ListControllers, "/controller_manager/list_controllers")
        if not self.list_controllers.wait_for_service(timeout.nanoseconds/1e9):
            self.fail("Service list_controllers not available")

        self.target_pose_pub = self.node.create_publisher(PoseStamped, "target_frame", 3)
        self.target_wrench_pub = self.node.create_publisher(WrenchStamped, "target_wrench", 3)
        self.ft_sensor_wrench_pub = self.node.create_publisher(WrenchStamped, "ft_sensor_wrench", 3)

    def test_nan_handling(self):
        """ Test whether every controller's state remains unchanged after processing NaN inputs

        The controllers' input callbacks should also work during the inactive state (=default) at startup.
        """
        # Target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = float('nan')
        target_pose.pose.position.y = float('nan')
        target_pose.pose.position.z = float('nan')
        target_pose.pose.orientation.x = float('nan')
        target_pose.pose.orientation.y = float('nan')
        target_pose.pose.orientation.z = float('nan')
        target_pose.pose.orientation.w = float('nan')
        self.target_pose_pub.publish(target_pose)

        # Force-torque sensor
        ft_sensor_wrench = WrenchStamped()
        ft_sensor_wrench.wrench.force.x = float('nan')
        ft_sensor_wrench.wrench.force.y = float('nan')
        ft_sensor_wrench.wrench.force.z = float('nan')
        ft_sensor_wrench.wrench.torque.x = float('nan')
        ft_sensor_wrench.wrench.torque.y = float('nan')
        ft_sensor_wrench.wrench.torque.z = float('nan')
        self.ft_sensor_wrench_pub.publish(ft_sensor_wrench)

        # Target wrench
        target_wrench = WrenchStamped()
        target_wrench.wrench.force.x = float('nan')
        target_wrench.wrench.force.y = float('nan')
        target_wrench.wrench.force.z = float('nan')
        target_wrench.wrench.torque.x = float('nan')
        target_wrench.wrench.torque.y = float('nan')
        target_wrench.wrench.torque.z = float('nan')
        self.target_wrench_pub.publish(target_wrench)

        time.sleep(3) # give the controllers some time to react

        for name in self.our_controllers:
            self.assertTrue(self.check_state(name, 'inactive'), f"{name} survives NaNs")

    def check_state(self, controller, state):
        """ Check the controller's state

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
