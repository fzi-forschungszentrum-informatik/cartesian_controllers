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
# \file    pose.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2022/11/09
#
# -----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
import numpy as np
import quaternion
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
import tf2_ros
import sys
import time
import threading
import os


class converter(Node):
    """Convert Twist messages to PoseStamped

    Use this node to integrate twist messages into a moving target pose in
    Cartesian space.  An initial TF lookup assures that the target pose always
    starts at the robot's end-effector.
    """

    def __init__(self):
        super().__init__("converter")

        self.twist_topic = self.declare_parameter("twist_topic", "my_twist").value
        self.pose_topic = self.declare_parameter("pose_topic", "my_pose").value
        self.frame_id = self.declare_parameter("frame_id", "base_link").value
        self.end_effector = self.declare_parameter("end_effector", "tool0").value

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.rot = np.quaternion(0, 0, 0, 1)
        self.pos = [0, 0, 0]

        self.pub = self.create_publisher(PoseStamped, self.pose_topic, 3)
        self.sub = self.create_subscription(Twist, self.twist_topic, self.twist_cb, 1)
        self.last = time.time()

        self.startup_done = False
        period = 1.0 / self.declare_parameter("publishing_rate", 100).value
        self.timer = self.create_timer(period, self.publish)

        self.thread = threading.Thread(target=self.startup, daemon=True)
        self.thread.start()

    def startup(self):
        """Make sure to start at the robot's current pose"""
        # Wait until we entered spinning in the main thread.
        time.sleep(1)
        try:
            start = self.tf_buffer.lookup_transform(
                target_frame=self.frame_id,
                source_frame=self.end_effector,
                time=rclpy.time.Time(),
            )

        except (
            tf2_ros.InvalidArgumentException,
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException,
        ) as e:
            print(f"Startup failed: {e}")
            os._exit(1)

        self.pos[0] = start.transform.translation.x
        self.pos[1] = start.transform.translation.y
        self.pos[2] = start.transform.translation.z
        self.rot.x = start.transform.rotation.x
        self.rot.y = start.transform.rotation.y
        self.rot.z = start.transform.rotation.z
        self.rot.w = start.transform.rotation.w
        self.startup_done = True

    def twist_cb(self, data):
        """Numerically integrate twist message into a pose

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

        _, q = quaternion.integrate_angular_velocity(
            lambda _: (wx, wy, wz), 0, dt, self.rot
        )

        self.rot = q[-1]  # the last one is after dt passed

    def publish(self):
        if not self.startup_done:
            return
        try:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.frame_id
            msg.pose.position.x = self.pos[0]
            msg.pose.position.y = self.pos[1]
            msg.pose.position.z = self.pos[2]
            msg.pose.orientation.x = self.rot.x
            msg.pose.orientation.y = self.rot.y
            msg.pose.orientation.z = self.rot.z
            msg.pose.orientation.w = self.rot.w

            self.pub.publish(msg)
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
