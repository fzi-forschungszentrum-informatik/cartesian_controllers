#!/usr/bin/env python
import sys
import unittest
import time
import rospy
from controller_manager_msgs.srv import ListControllers
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from controller_manager_msgs.utils import get_rosparam_controller_names

PKG = 'cartesian_controller_tests'
NAME = 'test_startup'


class IntegrationTest(unittest.TestCase):
    """ An integration test for controller initialization and startup

    We check if each controller successfully performs the life cycle of:
    `init()` - `starting()` - `update()` - `stopping()`.
    """
    def __init__(self, *args):
        super(IntegrationTest, self).__init__(*args)

        # Configuration
        rospy.init_node(NAME)
        timeout = rospy.Duration(5)
        self.our_controllers = [
            'my_cartesian_motion_controller',
            'my_cartesian_force_controller',
            'my_cartesian_compliance_controller',
            'my_motion_control_handle',
        ]

        self.invalid_controllers = [
            'invalid_cartesian_force_controller',
            'invalid_cartesian_compliance_controller',
        ]

        # ROS Interfaces
        self.list_controllers = rospy.ServiceProxy('/controller_manager/list_controllers', ListControllers)
        try:
            self.list_controllers.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            self.fail("Service list_controllers not available: {}".format(err))

        self.switch_controller = rospy.ServiceProxy('/controller_manager/switch_controller', SwitchController)
        try:
            self.switch_controller.wait_for_service(timeout.to_sec())
        except rospy.exceptions.ROSException as err:
            self.fail("Service switch_controllers not available: {}".format(err))

        # Wait until controller spawner is done
        time.sleep(3)

    def test_controller_initialization(self):
        """ Test whether every controller got initialized correctly

        We check if the list of all controllers currently managed by the
        controller manager contains our controllers and if they have `state:
        initialized`.
        """
        for name in self.our_controllers:
            self.assertTrue(self.check_state(name, 'initialized'), "{} is initialized correctly".format(name))

    def test_invalid_controller_initialization(self):
        """ Test whether the invalid controllers' initialization fails as expected

        This is the case when:
        1) The parameter server has the invalid controllers.
           (We have previously loaded them in the test launch files and spawned them with `--stopped`).
        2) The list of controllers managed by the controller manager does not contain the invalid controllers
        """
        for name in self.invalid_controllers:
            self.assertTrue(self.check_parameter_server(name), "{} was loaded as expected".format(name))

        for name in self.invalid_controllers:
            self.assertFalse(self.check_state(name, 'initialized'), "{} initializes although it should not".format(name))

    def test_controller_switches(self):
        """ Test whether every controller starts, runs, and stops correctly

        We start each of our controllers individually and check if its state is
        `running`.  We then switch it off and check whether its state is
        `stopped`.
        """
        for name in self.our_controllers:
            self.start_controller(name)
            self.assertTrue(self.check_state(name, 'running'), "{} is starting correctly".format(name))
            time.sleep(1) # process some update() cycles
            self.stop_controller(name)
            self.assertTrue(self.check_state(name, 'stopped'), "{} is stopping correctly".format(name))

    def check_state(self, controller, state):
        """ Check the controller's state

        Return True if the controller's state is `state`, else False.
        Return False if the controller is not listed.
        """
        listed_controllers = self.list_controllers()
        for entry in listed_controllers.controller:
            if entry.name == controller:
                return True if entry.state == state else False
        return False

    def check_parameter_server(self, controller):
        """ Check if the controller is in the parameter server """
        for name in get_rosparam_controller_names("/"):
            if name == controller:
                return True
        return False

    def start_controller(self, controller):
        """ Start the given controller with a best-effort strategy """
        srv = SwitchControllerRequest()
        srv.start_controllers = [controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller(srv)

    def stop_controller(self, controller):
        """ Stop the given controller with a best-effort strategy """
        srv = SwitchControllerRequest()
        srv.stop_controllers = [controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller(srv)

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
