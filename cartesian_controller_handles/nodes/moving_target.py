#!/usr/bin/env python
# - BEGIN LICENSE BLOCK -------------------------------------------------------
# - END LICENSE BLOCK ---------------------------------------------------------

# -----------------------------------------------------------------------------
# \file    moving_target.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2018/09/12
#
# -----------------------------------------------------------------------------

import rospy

class MovingTarget(object):
    """ Turn a TF frame into a moving target for CartesianMotionControllers

    This class creates an interactive marker to a (possibly moving) TF frame
    and publishes its pose as a PoseStamped with respect to a specified robot
    base. Its principal application is to create dynamic targets for
    CartesianMotionControllers, since they cannot follow TFs directly. The user
    can adjust offsets manually with the drag-and-drop marker in RViz.

    On startup, the marker coincides with the specified robot end-effector to
    prevent jumps in the CartesianMotionControllers. The marker can be reset
    to the robot end-effector at all times via the 'reset' service call.
    """

    def __init__(self):
        rospy.init_node('moving_target', anonymous=True)
        pass

if __name__ == '__main__':
    _ = MovingTarget()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
