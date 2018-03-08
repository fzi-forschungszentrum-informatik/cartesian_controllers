# Spacenav To Wrench
A small helper node to re-publish *geometry_msgs/Twist* messages
as *geometry_msgs/WrenchStamped* messages.
It turns the *spacenav_node* from the *joystick_drivers* package into a force control input device.
Adjust the launch file as needed.

## Usage
```bash
roslaunch spacenav_to_wrench spacenav_to_wrench.launch
```
