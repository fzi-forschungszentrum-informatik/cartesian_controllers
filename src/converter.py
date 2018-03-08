#!/usr/bin/env python

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

        self.pub = rospy.Publisher(self.wrench_topic, WrenchStamped, queue_size=3)
        self.sub = rospy.Subscriber(self.twist_topic, Twist, self.republish)


    def republish(self,data):
        wrench = WrenchStamped()
        wrench.header.stamp     = rospy.Time.now()
        wrench.header.frame_id  = self.frame_id
        wrench.wrench.force.x   = data.linear.x
        wrench.wrench.force.y   = data.linear.y
        wrench.wrench.force.z   = data.linear.z
        wrench.wrench.torque.x  = data.angular.x
        wrench.wrench.torque.y  = data.angular.y
        wrench.wrench.torque.z  = data.angular.z
        self.pub.publish(wrench)


if __name__ == '__main__':
    conv = converter()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
