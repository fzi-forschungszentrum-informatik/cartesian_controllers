## Cartesian Compliance Controller ##

This package provides a ros-controller which implements [Forward Dynamics Compliance Control (FDCC)](https://ieeexplore.ieee.org/document/8206325) on a set of joints.
It's essentially a hybrid of the `CartesianMotionController` and the `CartesianForceController`, and offers both interfaces for control.

## Getting Started
1) In a sourced terminal, run
```bash
roslaunch cartesian_controller_examples examples.launch
```
2) In another sourced terminal, open rqt and navigate to the *Controller Manager* plugin under *Robot Tools*.
Select */controller_manager* as namespace and activate *my_cartesian_compliance_controller*.

3) Publish a geometry_msgs/WrenchStamped to */target_wrench* with force x = 10 and watch the robot move.

4) In rqt open the *Dynamic Reconfigure* plugin under *Configuration*. Play a
little with the parameters of *my_cartesian_compliance_controller* (e.g. stiffness/trans_x) and observe the
effect of the target_wrench in RViz.

5) In rqt in the *Controller Manager*, activate *my_motion_control_handle*.

6) Switch to the RViz window and tick the box *Interactive Markers*. Use the
interactive marker to guide the end effector of the robot. Adjust the solver's
parameters in *Dynamic Reconfigure*, such as `error_scale` and get a feeling
how it responds.


## Example Configuration
Below is an example entry for a controller specific configuration. Also see cartesian_controller_examples/config/example_controllers.yaml for further tips.
```yaml
my_cartesian_compliance_controller:
    type: "position_controllers/CartesianComplianceController"
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    ft_sensor_ref_link: "sensor_link"
    compliance_ref_link: "tool0"
    target_frame_topic: "target_frame"
    joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6

    stiffness:
        trans_x: 500
        trans_y: 500
        trans_z: 500
        rot_x: 20
        rot_y: 20
        rot_z: 20

    solver:
        error_scale: 0.5

    pd_gains:
        trans_x: {p: 0.05}
        trans_y: {p: 0.05}
        trans_z: {p: 0.05}
        rot_x: {p: 1.50}
        rot_y: {p: 1.50}
        rot_z: {p: 1.50}
```

A minimal example can be found in *cartesian_controller_examples* of this meta package.
Also check the top-level **README.md** for further information.
