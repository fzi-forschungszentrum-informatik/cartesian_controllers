#!/usr/bin/env python
import sys
import unittest
import time
import rospy
from controller_manager_msgs.srv import ListControllers

PKG = 'cartesian_controller_tests'
NAME = 'test_startup'


class IntegrationTest(unittest.TestCase):
    def __init__(self, *args):
        super(IntegrationTest, self).__init__(*args)

        rospy.init_node(NAME)

        timeout = rospy.Duration(5)

        self.list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        try:
            self.list_controllers.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            self.fail("Service list_controllers not available: {}".format(err))

    def test_controller_initialization(self):
        """ Test whether every controller got initialized correctly

        We check if the list of all controllers currently managed by the
        controller manager contains our controllers and if they have `state:
        initialized`.
        """
        our_controllers = {
            'my_cartesian_motion_controller': False,
            'my_cartesian_force_controller': False,
            'my_cartesian_compliance_controller': False,
            'my_motion_control_handle': False,
        }
        listed_controllers = self.list_controllers()
        for name in our_controllers.keys():
            for entry in listed_controllers.controller:
                if entry.name == name:
                    our_controllers[name] = True if entry.state == 'initialized' else False

        for key, value in our_controllers.items():
            self.assertTrue(value, "{} initialized".format(key))


if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
