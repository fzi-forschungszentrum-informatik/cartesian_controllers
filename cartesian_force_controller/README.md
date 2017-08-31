## Cartesian Force Controller ##

This package provides a ros-controller which implements free-floating force control on a set of joints.

## Example ##
```yaml
my_controller_name:
    type: "position_controllers/CartesianForceController"
    end_effector_link: "my_end_effector"
    robot_base_link: "base_link"
    ft_sensor_ref_link: "my_sensor"
    joints:
    - joint_name_1
    - joint_name_2
    - joint_name_3
    - joint_name_4
    - joint_name_5
    - joint_name_6

    damping:
        trans: 100
        rot: 100

    pid_gains:
        trans_x: {p: 50.0, i: 0, d: 0}
        trans_y: {p: 50.0, i: 0, d: 0}
        trans_z: {p: 50.0, i: 0, d: 0}
        rot_x: {p: 50.0, i: 0, d: 0}
        rot_y: {p: 50.0, i: 0, d: 0}
        rot_z: {p: 50.0, i: 0, d: 0}
```
