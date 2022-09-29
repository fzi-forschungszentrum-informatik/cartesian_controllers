#!/usr/bin/env python3

# ROS related
import rospy
from sensor_msgs.msg import Joy

# Other
import subprocess
import numpy as np

class buttons:
    """ React to button events """

    def __init__(self):
        rospy.init_node('spacenav_buttons', anonymous=False)

        self.repeat_same_button = rospy.get_param('~repeat_same_button')
        self.button_sleep = rospy.get_param('~button_sleep')
        self.button_cmds = rospy.get_param('~button_cmds')
        self.cmd_dirs = rospy.get_param('~cmd_dirs')
        self.last_button_cmds = None

        self.joystick_topic = rospy.get_param('~joystick_topic',default="my_joystick_topic")
        self.sub = rospy.Subscriber(self.joystick_topic, Joy, self.event_callback, queue_size=1)

    def event_callback(self,data):
        # Have some buttons been repeatedly pressed?
        if self.last_button_cmds and any(np.bitwise_and(data.buttons,self.last_button_cmds)):
            return
        for idx, val in enumerate(data.buttons):
            if val == 1:
                exec_dir = self.cmd_dirs[idx]
                if not exec_dir:    # Empty string
                    exec_dir = None
                subprocess.Popen(
                    self.button_cmds[idx],
                    stdin=subprocess.PIPE,
                    cwd=exec_dir,
                    shell=True)
                # Prevent pressing the same buttons in a row
                if not self.repeat_same_button:
                    self.last_button_cmds = data.buttons
                rospy.sleep(self.button_sleep)


if __name__ == '__main__':
    _ = buttons()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
