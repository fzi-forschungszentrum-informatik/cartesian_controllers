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

# ROS
import rospy
from geometry_msgs.msg import Pose, PoseStamped
from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from std_srvs.srv import Trigger
import tf

# Other
import numpy as np
import copy

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

        # Configuration
        self.robot_base = rospy.get_param('~robot_base_link')
        self.end_effector = rospy.get_param('~end_effector_link')
        self.moving_tf_frame = rospy.get_param('~moving_tf_frame')
        self.target_frame_topic = rospy.get_param('~target_frame_topic')
        self.publish_rate = rospy.Rate(rospy.get_param('~publish_rate'))

        # TF, topics and services
        self.tf_listener = tf.TransformListener()
        self.reset_service = rospy.Service("~reset", Trigger, self.reset_callback)
        self.target_pose_pub = rospy.Publisher(self.target_frame_topic, PoseStamped, queue_size=3)

        # Startup
        starting_pose = self.get_end_effector_pose()
        if starting_pose is None:
            rospy.signal_shutdown("Initial starting pose invalid")
        self.marker = self.make_interactive_marker(self.moving_tf_frame, starting_pose)

        self.server = InteractiveMarkerServer("moving_target")
        self.server.insert(self.marker, self.process_marker_feedback)
        self.server.applyChanges()

    def process_marker_feedback(self, feedback):
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            self.marker.pose = feedback.pose

    def publish_target_pose(self):
        # Marker w.r.t. the moving tf reference
        pose = PoseStamped()
        pose.header.frame_id = self.moving_tf_frame
        pose.header.stamp = rospy.Time(0)
        pose.pose = self.marker.pose

        # Marker w.r.t. the robot base
        try:
            self.tf_listener.waitForTransform(  # Returns at once if transform exists
                    self.robot_base,
                    self.moving_tf_frame,
                    rospy.Time(0),
                    timeout=rospy.Duration(1)
                    )
            pose_in_base = self.tf_listener.transformPose(
                    self.robot_base,
                    pose)
        except tf.Exception as e:
            rospy.logwarn("{}".format(e))
            return

        self.target_pose_pub.publish(pose_in_base)
        self.publish_rate.sleep()

    def reset_callback(self, req):
        pose = self.get_end_effector_pose()
        if pose is None:
            return {'success': False, 'message': "Reset pose is invalid"}
        else:
            self.server.setPose(self.marker.name, pose)
            self.server.applyChanges()
            return {'success': True, 'message': "Reset to {}".format(self.end_effector)}

    def add_marker_visualization(self, marker, scale):
        # Create a sphere as a handle
        visual = Marker()
        visual.type = Marker.SPHERE
        visual.scale.x = scale  # bounding box in meter
        visual.scale.y = scale
        visual.scale.z = scale
        visual.color.r = 1.0  # orange
        visual.color.g = 0.5
        visual.color.b = 0.0
        visual.color.a = 1.0

        # Create a non-interactive control for the appearance
        visual_control = InteractiveMarkerControl()
        visual_control.always_visible = True
        visual_control.markers.append(visual)
        marker.controls.append(visual_control)

    def add_axis_control(self, marker, axis):
        control = InteractiveMarkerControl()
        norm = np.linalg.norm(axis)
        if norm == 0:
            return
        control.orientation.w = 1 / norm
        control.orientation.x = axis[0] / norm
        control.orientation.y = axis[1] / norm
        control.orientation.z = axis[2] / norm

        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(copy.deepcopy(control))
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(copy.deepcopy(control))

    def prepare_marker_controls(self, marker):
        # Add colored sphere as visualization
        self.add_marker_visualization(marker, 0.05)

        # Create move and rotate controls along all axis
        self.add_axis_control(marker, [1, 0, 0]);
        self.add_axis_control(marker, [0, 1, 0]);
        self.add_axis_control(marker, [0, 0, 1]);

    def get_end_effector_pose(self):
        try:
            self.tf_listener.waitForTransform(
                    self.moving_tf_frame,
                    self.end_effector,
                    rospy.Time(0),
                    timeout=rospy.Duration(1),
                    polling_sleep_duration=rospy.Duration(0.1)
                    )
            pos, rot = self.tf_listener.lookupTransform(
                    self.moving_tf_frame,
                    self.end_effector,
                    rospy.Time(0))
        except tf.Exception as e:
            rospy.logwarn("{}".format(e))
            return None

        pose = Pose()
        pose.position.x = pos[0]
        pose.position.y = pos[1]
        pose.position.z = pos[2]
        pose.orientation.x = rot[0]
        pose.orientation.y = rot[1]
        pose.orientation.z = rot[2]
        pose.orientation.w = rot[3]
        return pose

    def make_interactive_marker(self, tf_reference_frame, starting_pose):

        marker = InteractiveMarker()
        marker.header.frame_id = tf_reference_frame
        marker.header.stamp = rospy.Time(0)
        marker.pose = starting_pose
        marker.scale = 0.1
        marker.name = "moving_offset"
        marker.description = "Follow {} with an offset".format(tf_reference_frame)

        self.prepare_marker_controls(marker)
        return marker

if __name__ == '__main__':
    moving_target = MovingTarget()
    try:
        while not rospy.is_shutdown():
            moving_target.publish_target_pose()
    except rospy.ROSInterruptException:
        pass
