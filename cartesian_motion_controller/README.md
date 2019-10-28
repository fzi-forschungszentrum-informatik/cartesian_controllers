# Cartesian Motion Controller
This package provides a controller for following Cartesian target poses.

This type of controller automatically interpolates between two target poses.
The amount of how much it interpolates can be set with
* the PD gains individually for each Cartesian axis
* the error_scale for all axes alike
* the number of iterations per control cycle.

As an example, the *CartesianMotionController* can be given target poses,
that pop up somewhere with a relatively low frequency.
Depending on how responsive the system is, users can obtain anything in between
fast jumps or smooth, directed motion, depending on their use case.

## Getting Started
1) In a sourced terminal, run
```bash
roslaunch cartesian_controller_examples examples.launch
```
2) In another sourced terminal, open rqt and navigate to the *Controller Manager* plugin under *Robot Tools*.
Select */controller_manager* as namespace and activate *my_cartesian_motion_controller*. Also activate
*my_motion_control_handle*.

3) Switch to the RViz window and tick the box *Interactive Markers*. Use the interactive marker to guide the end effector of the robot. The robot should follow slowely.

4) In rqt open the *Dynamic Reconfigure* plugin under *Configuration*. Play a
little with the parameters of *my_cartesian_motion_controller* (e.g. solver/error_scale) and observe the
changes in RViz  using the interactive marker as handle.

## Example Configuration
Below is an example entry for a controller specific configuration. Also see cartesian_controller_examples/config/example_controllers.yaml for further tips.
```yaml
my_cartesian_motion_controller:
    type: "position_controllers/CartesianMotionController"
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    target_frame_topic: "target_frame"
    joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6

    pd_gains:
        trans_x: {p: 10.0}
        trans_y: {p: 10.0}
        trans_z: {p: 10.0}
        rot_x: {p: 1.0}
        rot_y: {p: 1.0}
        rot_z: {p: 1.0}
```

The controller configuration must be loaded to the ros parameter server and is accessed by the controller manager when looking for configuration for the loaded controller *my_cartesian_motion_controller*.

## Tips
Note, that the maximal joint velocities of the robot usually represent the
limiting factor for speed.
If you notice that increasing *error_scale* does
not make the robot move as fast as you would like, then probably the joint
velocity limits in your ROS controlled hardware interface are already reached for this motion.
Reaching these limits on individual joints also leads to a distortion of the
Cartesian path.
Also note that when using this controller in inverse kinematics mode (very high gains), then
you must also publish high-frequently sampled targets in order to avoid jumps on joint
control level.
