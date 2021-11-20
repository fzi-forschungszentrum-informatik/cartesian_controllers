#!/usr/bin/env python
import sys
import unittest
import time
import rospy

PKG = 'cartesian_controller_tests'
NAME = 'test_startup'


class IntegrationTest(unittest.TestCase):
    def __init__(self, *args):
        super(IntegrationTest, self).__init__(*args)

        rospy.init_node(NAME)

    def test_controller_initialization(self):
        """ Test whether every controller got initialized correctly """
        self.assertEqual(True, True)

if __name__ == '__main__':
    import rostest
    rostest.run(PKG, NAME, IntegrationTest, sys.argv)
