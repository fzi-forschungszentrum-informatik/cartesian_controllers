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
# \file    buttons.py
#
# \author  Stefan Scherzinger <scherzin@fzi.de>
# \date    2022/11/09
#
# -----------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
import subprocess
import numpy as np
import sys
import time


class buttons(Node):
    """React to button events"""

    def __init__(self):
        super().__init__("spacenav_buttons")

        self.repeat_same_button = self.declare_parameter(
            "repeat_same_button", False
        ).value
        self.button_sleep = self.declare_parameter("button_sleep", 0.1).value
        self.button_cmds = self.declare_parameter("button_cmds", [""]).value
        self.cmd_dirs = self.declare_parameter("cmd_dirs", [""]).value
        self.last_button_cmds = None

        self.joystick_topic = self.declare_parameter("joystick_topic", "").value
        self.sub = self.create_subscription(
            Joy, self.joystick_topic, self.event_callback, 1
        )

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


def main(args=None):
    rclpy.init(args=args)

    node = buttons()
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
