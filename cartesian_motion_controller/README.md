## Cartesian Motion Controller ##

This package provides a controller for mapping Cartesian end-effector motion to joint control commands of a set of rotary joints (kinematic chain).

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
       rot_x: {p: 0.020, i: 0, d: 0.0001}
       rot_y: {p: 0.020, i: 0, d: 0.0001}
       rot_z: {p: 0.020, i: 0, d: 0.0001}
       trans_x: {p: 0.020, i: 0, d: 0.0001}
       trans_y: {p: 0.020, i: 0, d: 0.0001}
       trans_z: {p: 0.020, i: 0, d: 0.0001}
```

This configuration must be loaded to the ros parameter server and is accessed by the controller manager when looking for configuration for the loaded controller *my_cartesian_motion_controller*.
A minimal example can be found in *test/test.launch* of this package.
