#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped


class converter:
    """ Convert Twist messages to WrenchStamped """

    def __init__(self):
        rospy.init_node('converter', anonymous=False)

        self.twist_topic = rospy.get_param('~twist_topic',default="my_twist")
        self.wrench_topic = rospy.get_param('~wrench_topic',default="my_wrench")
        self.frame_id = rospy.get_param('~frame_id',default="world")
        self.rate = rospy.Rate(rospy.get_param('~publishing_rate',default=100))

        self.buffer = WrenchStamped()

        self.pub = rospy.Publisher(self.wrench_topic, WrenchStamped, queue_size=3)
        self.sub = rospy.Subscriber(self.twist_topic, Twist, self.twist_cb)


    def twist_cb(self,data):
        self.buffer.header.stamp     = rospy.Time.now()
        self.buffer.header.frame_id  = self.frame_id
        self.buffer.wrench.force.x   = data.linear.x
        self.buffer.wrench.force.y   = data.linear.y
        self.buffer.wrench.force.z   = data.linear.z
        self.buffer.wrench.torque.x  = data.angular.x
        self.buffer.wrench.torque.y  = data.angular.y
        self.buffer.wrench.torque.z  = data.angular.z


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
