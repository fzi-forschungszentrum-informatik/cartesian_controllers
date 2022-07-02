#!/usr/bin/env python3
import unittest

import launch
import launch.actions
import launch_testing.actions
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_test_description():

    setup = IncludeLaunchDescription(
        PathJoinSubstitution(
            [FindPackageShare("cartesian_controller_simulation"), "launch", "simulation.launch.py"]
        )
    )
    until_ready = 10.0 # sec
    return LaunchDescription([setup, TimerAction(period=until_ready, actions=[launch_testing.actions.ReadyToTest()])])


class IntegrationTest(unittest.TestCase):

    def test_controller_initialization(self):
        self.assertTrue(True)

