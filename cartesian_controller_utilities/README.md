# Cartesian Controller Utilities
A helper package with additional functionality for the `spacenav_node`.

This package turns the *spacenav_node* from the
[joystick_drivers](https://github.com/ros-drivers/joystick_drivers) into a
force and motion control input device for teleoperation with the `cartesian_controllers`:

- **Force control**: Re-publish `geometry_msgs/Twist` messages as `geometry_msgs/WrenchStamped` messages to intuitively control the `target_wrench`.
- **Motion control**: Turn `geometry_msgs/Twist` messages into a `geometry_msgs/PoseStamped` to smoothly steer the robot via its `target_frame`.
- **Button commands**: Define custom commands for the spacenav buttons with ROS command line instructions. You can trigger events, such as calling ROS services and publish to topics.

## Usage
Load the spacenav device with
```bash
roslaunch cartesian_controller_utilities spacenav.launch
```
Depending on your use case, call one (or several) of these:
```bash
roslaunch cartesian_controller_utilities spacenav_to_wrench.launch
roslaunch cartesian_controller_utilities spacenav_to_pose.launch
roslaunch cartesian_controller_utilities buttons_events.launch
```
Also check the configuration parameters in each `.launch` file.

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
More information can be found [here](http://answers.gazebosim.org/question/14225/how-can-i-turn-off-the-space-navigators-control-of-the-camera/).
