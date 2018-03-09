# Spacenav To Wrench
A small helper node to re-publish *geometry_msgs/Twist* messages
as *geometry_msgs/WrenchStamped* messages.
It turns the *spacenav_node* from the *joystick_drivers* package into a force control input device.
Adjust the launch file as needed.

## Usage
```bash
roslaunch spacenav_to_wrench spacenav_to_wrench.launch
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
