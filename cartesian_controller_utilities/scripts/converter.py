#!/usr/bin/env python3
################################################################################
# Copyright 2022 FZI Research Center for Information Technology
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from this
# software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
################################################################################

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
