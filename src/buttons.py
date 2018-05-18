#!/usr/bin/env python

# ROS related
import rospy
from sensor_msgs.msg import Joy

# Other
from subprocess import call

class buttons:
    """ React to button events """

    def __init__(self):
        rospy.init_node('spacenav_buttons', anonymous=False)

        self.joystick_topic = rospy.get_param('~joystick_topic',default="my_joystick_topic")
        self.sub = rospy.Subscriber(self.joystick_topic, Joy, self.event_callback)

        self.button_cmds = rospy.get_param('button_cmds')
        self.last_button_cmds = None


    def event_callback(self,data):
        if data.buttons == self.last_button_cmds:
            return
        for idx, val in enumerate(data.buttons):
            if val == 1:
                # Prevent pressing the same buttons in a row
                call(self.button_cmds[idx],shell=True)
                self.last_button_cmds = data.buttons
                return


if __name__ == '__main__':
    _ = buttons()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
