#!/usr/bin/env python3

import numpy as np
import quaternion
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf2_ros
import sys
import time


class converter:
    """ Convert Twist messages to PoseStamped

    Use this node to integrate twist messages into a moving target pose in
    Cartesian space.  An initial TF lookup assures that the target pose always
    starts at the robot's end-effector.
    """

    def __init__(self):
        rospy.init_node('converter', anonymous=False)

        self.twist_topic = rospy.get_param('~twist_topic', default="my_twist")
        self.pose_topic = rospy.get_param('~pose_topic', default="my_pose")
        self.frame_id = rospy.get_param('~frame_id', default="base_link")
        self.end_effector = rospy.get_param('~end_effector', default="tool0")
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate', default=100))

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(3.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self.rot = np.quaternion(0, 0, 0, 1)
        self.pos = [0, 0, 0]

        # Start where we are
        if not self.startup():
            sys.exit(0)

        self.pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=3)
        self.sub = rospy.Subscriber(self.twist_topic, Twist, self.twist_cb)
        self.last = time.time()

    def startup(self):
        try:
            start = self.tf_buffer.lookup_transform(
                target_frame=self.frame_id, source_frame=self.end_effector, time=rospy.Time(0), timeout=rospy.Duration(5))

        except (tf2_ros.InvalidArgumentException, tf2_ros.LookupException,
                tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(e)
            return False

        self.pos[0] = start.transform.translation.x
        self.pos[1] = start.transform.translation.y
        self.pos[2] = start.transform.translation.z
        self.rot.x = start.transform.rotation.x
        self.rot.y = start.transform.rotation.y
        self.rot.z = start.transform.rotation.z
        self.rot.w = start.transform.rotation.w
        return True



    def twist_cb(self, data):
        """ Numerically integrate twist message into a pose

        Use global self.frame_id as reference for the navigation commands.
        """
        now = time.time()
        dt = now - self.last
        self.last = now

        # Position update
        self.pos[0] += data.linear.x * dt
        self.pos[1] += data.linear.y * dt
        self.pos[2] += data.linear.z * dt

        # Orientation update
        wx = data.angular.x
        wy = data.angular.y
        wz = data.angular.z

        _, q = quaternion.integrate_angular_velocity(lambda _: (wx, wy, wz), 0, dt, self.rot)

        self.rot = q[-1]  # the last one is after dt passed

    def publish(self):
        if not rospy.is_shutdown():
            msg = PoseStamped()
            msg.header.stamp = rospy.Time.now()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = self.pos[0]
            msg.pose.position.y = self.pos[1]
            msg.pose.position.z = self.pos[2]
            msg.pose.orientation.x = self.rot.x
            msg.pose.orientation.y = self.rot.y
            msg.pose.orientation.z = self.rot.z
            msg.pose.orientation.w = self.rot.w

            try:
                self.pub.publish(msg)
            except rospy.ROSException:
                # Swallow 'publish() to closed topic' error.
                # This rarely happens on killing this node.
                pass


if __name__ == '__main__':
    conv = converter()
    try:
        while not rospy.is_shutdown():
            conv.publish()
            conv.rate.sleep()
    except rospy.ROSInterruptException:
        pass
