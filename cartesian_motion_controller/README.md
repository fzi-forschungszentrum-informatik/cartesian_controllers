## Cartesian Motion Controller ##

This package provides a controller for mapping Cartesian end-effector motion to joint control commands of a set of joints.
The controller tracks a given tf target frame. The responsiveness can be adjusted with the controller gains.

## Example ##
Below is an example entry for a controller specific configuration.
```yaml
my_cartesian_motion_controller:
    type: "position_controllers/CartesianMotionController"
    end_effector_link: "my_end_effector_link"
    robot_base_link: "link_before_first_actuated_joint"
    target_frame: "my_target"
    joints:
    - joint_name_1
    - joint_name_2
    - joint_name_3
    - joint_name_4
    - joint_name_5
    - joint_name_6

    pid_gains:
       trans_x: {p: 50.0, i: 0, d: 0}
       trans_y: {p: 50.0, i: 0, d: 0}
       trans_z: {p: 50.0, i: 0, d: 0}
       rot_x: {p: 10.0, i: 0, d: 0}
       rot_y: {p: 10.0, i: 0, d: 0}
       rot_z: {p: 10.0, i: 0, d: 0}
```

This configuration must be loaded to the ros parameter server and is accessed by the controller manager when looking for configuration for the loaded controller *my_cartesian_motion_controller*.
A minimal example can be found in *cartesian_controller_test* of this meta package.
