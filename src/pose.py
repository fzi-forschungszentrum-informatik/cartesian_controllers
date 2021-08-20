#!/usr/bin/env python3

import numpy as np
import quaternion
import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped


class converter:
    """ Convert Twist messages to PoseStamped

    Use this node to move a target pose in Cartesian space with the space
    mouse.
    """

    def __init__(self):
        rospy.init_node('converter', anonymous=False)

        self.twist_topic = rospy.get_param('~twist_topic', default="my_twist")
        self.pose_topic = rospy.get_param('~pose_topic', default="my_wrench")
        self.frame_id = rospy.get_param('~frame_id', default="world")
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate', default=100))

        self.buffer = PoseStamped()
        self.rot = np.quaternion(0, 0, 0, 1)
        self.pos = [0, 0, 0]

        self.pub = rospy.Publisher(self.pose_topic, PoseStamped, queue_size=3)
        self.sub = rospy.Subscriber(self.twist_topic, Twist, self.twist_cb)

    def twist_cb(self, data):
        """ Numerically integrate twist message into a pose

        Use global self.frame_id as reference for the navigation commands.
        """
        dt = 0.002
        scale = 0.01

        # Position update
        self.pos[0] += data.linear.x * dt * scale
        self.pos[1] += data.linear.y * dt * scale
        self.pos[2] += data.linear.z * dt * scale

        # Orientation update
        wx = data.angular.x
        wy = data.angular.y
        wz = data.angular.z

        _, q = quaternion.integrate_angular_velocity(lambda _: (wx, wy, wz), 0, dt, self.rot)

        self.rot = q[-1]  # the last one is after dt passed

        self.buffer.header.stamp = rospy.Time.now()
        self.buffer.header.frame_id = self.frame_id
        self.buffer.pose.position.x = self.pos[0]
        self.buffer.pose.position.y = self.pos[1]
        self.buffer.pose.position.z = self.pos[2]
        self.buffer.pose.orientation.x = self.rot.x
        self.buffer.pose.orientation.y = self.rot.y
        self.buffer.pose.orientation.z = self.rot.z
        self.buffer.pose.orientation.w = self.rot.w

    def publish(self):
        if not rospy.is_shutdown():
            try:
                self.pub.publish(self.buffer)
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
