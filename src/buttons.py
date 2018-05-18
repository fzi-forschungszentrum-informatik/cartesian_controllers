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


    def event_callback(self,data):
        for idx, val in enumerate(data.buttons):
            if val == 1:
                call(self.button_cmds[idx],shell=True)


if __name__ == '__main__':
    _ = buttons()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
