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

# -----------------------------------------------------------------------------
# \file    converter.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2022/11/09
#
# -----------------------------------------------------------------------------

import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped


class converter(Node):
    """Convert Twist messages to WrenchStamped"""

    def __init__(self):
        super().__init__("converter")

        self.twist_topic = self.declare_parameter("twist_topic", "my_twist").value
        self.wrench_topic = self.declare_parameter("wrench_topic", "my_wrench").value
        self.frame_id = self.declare_parameter("frame_id", "world").value
        period = 1.0 / self.declare_parameter("publishing_rate", 100).value
        self.timer = self.create_timer(period, self.publish)

        self.buffer = WrenchStamped()

        self.pub = self.create_publisher(WrenchStamped, self.wrench_topic, 3)
        self.sub = self.create_subscription(Twist, self.twist_topic, self.twist_cb, 1)

    def twist_cb(self, data):
        self.buffer.header.stamp = self.get_clock().now().to_msg()
        self.buffer.header.frame_id = self.frame_id
        self.buffer.wrench.force.x = data.linear.x
        self.buffer.wrench.force.y = data.linear.y
        self.buffer.wrench.force.z = data.linear.z
        self.buffer.wrench.torque.x = data.angular.x
        self.buffer.wrench.torque.y = data.angular.y
        self.buffer.wrench.torque.z = data.angular.z

    def publish(self):
        try:
            self.pub.publish(self.buffer)
        except Exception:
            # Swallow 'publish() to closed topic' error.
            # This rarely happens on killing this node.
            pass


def main(args=None):
    rclpy.init(args=args)
    node = converter()
    rclpy.spin(node)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        rclpy.shutdown()
        sys.exit(0)
    except Exception as e:
        print(e)
        sys.exit(1)
