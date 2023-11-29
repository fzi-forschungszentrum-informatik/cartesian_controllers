#!/usr/bin/env python3
import numpy as np
import rclpy
import subprocess
import sys
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from sensor_msgs.msg import Joy


class TeachDevice(Node):
    def __init__(self):
        super().__init__("teach_device")

        self.repeat_same_button = self.declare_parameter(
            "repeat_same_button", False
        ).value
        self.button_sleep = self.declare_parameter("button_sleep", 0.1).value
        self.button_cmds = self.declare_parameter("button_cmds", [""]).value
        self.cmd_dirs = self.declare_parameter("cmd_dirs", [""]).value
        self.joystick_topic = self.declare_parameter("joystick_topic", "").value
        self.twist_topic = self.declare_parameter("twist_topic", "my_twist").value
        self.wrench_topic = self.declare_parameter("wrench_topic", "my_wrench").value
        self.frame_id = self.declare_parameter("frame_id", "world").value
        period = 1.0 / self.declare_parameter("publishing_rate", 100).value

        self.last_button_cmds = None
        self.buffer = WrenchStamped()

        self.button_sub = self.create_subscription(
            Joy, self.joystick_topic, self.event_callback, 1
        )
        self.wrench_pub = self.create_publisher(WrenchStamped, self.wrench_topic, 3)
        self.twist_sub = self.create_subscription(
            Twist, self.twist_topic, self.twist_cb, 1
        )
        self.timer = self.create_timer(period, self.publish)

    def event_callback(self, data):
        # Have some buttons been repeatedly pressed?
        if self.last_button_cmds and any(
            np.bitwise_and(data.buttons, self.last_button_cmds)
        ):
            return
        for idx, val in enumerate(data.buttons):
            if val == 1:
                exec_dir = self.cmd_dirs[idx]
                if not exec_dir:  # Empty string
                    exec_dir = None
                subprocess.Popen(
                    self.button_cmds[idx],
                    stdin=subprocess.PIPE,
                    cwd=exec_dir,
                    shell=True,
                )
                # Prevent pressing the same buttons in a row
                if not self.repeat_same_button:
                    self.last_button_cmds = data.buttons
                time.sleep(self.button_sleep)

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
            self.wrench_pub.publish(self.buffer)
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TeachDevice()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        sys.exit(0)
    except Exception as e:
        print(e)
        sys.exit(1)
