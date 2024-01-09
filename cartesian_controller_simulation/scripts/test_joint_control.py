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

"""
Bring our test robot into a predefined joint configuration

This is a small helper to drive the robot between hard-coded joint configurations.
We can use this to tweak the joint controller gains.
"""
import numpy as np
from tkinter import Tk, Scale
import rclpy
from rclpy.node import Node
import subprocess
import sys

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


class JointGui(Node):
    def __init__(self):
        super().__init__("joint_gui")

        # Check the available ROS2 nodes for the joint trajectory controller.
        # We assume it's called /joint_trajectory_controller.
        nodes = subprocess.check_output(
            "ros2 node list", stderr=subprocess.STDOUT, shell=True
        )
        nodes = nodes.decode("utf-8").split("\n")
        if "/joint_trajectory_controller" in nodes:
            controller = "/joint_trajectory_controller"
        else:
            rclpy.shutdown()
            print("No suitable joint trajectory controller found.")
            sys.exit(0)

        # Control the hand with publishing joint trajectories.
        self.publisher = self.create_publisher(
            JointTrajectory, f"{controller}/joint_trajectory", 10
        )

        # Configuration presets
        self.joint_names = [
            "joint1",
            "joint2",
            "joint3",
            "joint4",
            "joint5",
            "joint6",
        ]
        self.fully_streched = [0, 0, 0, 0, 0, 0]
        self.home = [
            np.pi / 4,
            -np.pi / 2,
            np.pi / 2,
            -np.pi / 2,
            -np.pi / 2,
            2 * np.pi,
        ]

    def gui(self):
        """A GUI with a single slider for opening/closing the JointGui"""
        self.percent = 100
        self.window = Tk()
        self.window.title("joint_gui")
        self.slider = Scale(self.window, from_=self.percent, to=0)
        self.slider.set(0)
        self.slider.bind("<ButtonRelease-1>", self.slider_changed)
        self.slider.pack()
        self.window.mainloop()

    def slider_changed(self, event):
        self.publish(self.slider.get())
        rclpy.spin_once(self, timeout_sec=0)

    def publish(self, opening):
        """Publish a new joint trajectory with an interpolated state

        We scale linearly with opening=[0,1] between `fully_streched` and `home`.
        """
        jpos = opening * np.array(self.home) + (self.percent - opening) * np.array(
            self.fully_streched
        )
        jpos = jpos / self.percent
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        jtp = JointTrajectoryPoint()
        jtp.positions = jpos.tolist()
        jtp.time_from_start = Duration(sec=2)
        msg.points.append(jtp)
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    gui = JointGui()
    gui.gui()
    gui.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
