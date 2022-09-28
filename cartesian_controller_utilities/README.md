# Cartesian Controller Utilities
A helper package with additional functionality for the `spacenav_node`.

This package turns the *spacenav_node* from the
[joystick_drivers](https://github.com/ros-drivers/joystick_drivers) into a
force and motion control input device for teleoperation with the `cartesian_controllers`:

- **Force control**: Re-publish `geometry_msgs/Twist` messages as `geometry_msgs/WrenchStamped` messages to intuitively control the `target_wrench`.
- **Motion control**: Turn `geometry_msgs/Twist` messages into a `geometry_msgs/PoseStamped` to smoothly steer the robot via its `target_frame`.
- **Button commands**: Define custom commands for the spacenav buttons with ROS command line instructions. You can trigger events, such as calling ROS services and publish to topics.

## Install
We need some additional system dependencies
```bash
sudo apt install libspnav-dev spacenavd
```
and the package for the space mouse driver. Install that for your ROS distribution, for instance with
```bash
sudo apt install ros-noetic-spacenav-node
```

We also need these Python dependencies for the scripts:
```bash
pip3 install numpy numpy-quaternion
```

## Usage
Launch the spacenav device with
```bash
roslaunch cartesian_controller_utilities spacenav.launch
```

There are additional `.launch` files for each use case:

### Force control
Start the simulation and the force controller as described [here](../cartesian_force_controller/README.md).
In a sourced terminal, call
```bash
roslaunch cartesian_controller_utilities spacenav_to_wrench.launch
```
You can now steer the robot's end-effector with the space mouse.
There's a parameter to change the coordinate frame for control: In rqt open the *Dynamic Reconfigure* plugin under *Configuration*,
navigate to *my_cartesian_force_controller* and untick the box for `hand_frame_control`. You now steer the robot in the controller's `robot_base_link` coordinates.


### Motion control


### Button commands


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
