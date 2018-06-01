# Spacenav Utils
A helper package to bring some additional functionality to the *spacenav_node* package.

This package is meant to turn the *spacenav_node* from the *joystick_drivers* package into a force control input device.
Re-publish *geometry_msgs/Twist* messages
as *geometry_msgs/WrenchStamped* messages and define custom commands for the buttons.
For example could you trigger the recording of rosbags with the spacenav buttons. Check the *etc/button_cmds.yaml*.

Adjust the launch files as needed.

## Usage
```bash
roslaunch spacenav_utils spacenav_to_wrench.launch
roslaunch spacenav_utils buttons_events.launch
```
## Disable GUI control in Gazebo
When working with the 3DConnexion as a force control input device, you probably don't want to control gazebo's camera at the same time. Remember your kernel drivers still treat your force control thing as a space mouse, which is used by gazebo by default. To switch this behavior off add the following lines to the *~/.gazebo/gui.ini* file:
```
[spacenav]
deadband_x = 0.1
deadband_y = 0.1
deadband_z = 0.1
deadband_rx = 0.1
deadband_ry = 0.1
deadband_rz = 0.1
topic=~/spacenav/remapped_joy_topic_to_something_not_used
```
More information can be found here:
  http://answers.gazebosim.org/question/14225/how-can-i-turn-off-the-space-navigators-control-of-the-camera/
