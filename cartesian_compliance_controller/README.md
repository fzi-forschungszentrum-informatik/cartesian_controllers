## Cartesian Compliance Controller ##

This package provides a ros-controller which implements Forward Dynamics Compliance Control (FDCC) [Scherzinger2017] on a set of joints.

## Example ##
```yaml
my_controller_name:
    type: "position_controllers/CartesianComplianceController"
    end_effector_link: "my_end_effector"
    robot_base_link: "base_link"
    compliance_ref_link: "compliance_center"
    ft_sensor_ref_link: "my_sensor"
    target_frame: "my_target"
    joints:
    - joint_name_1
    - joint_name_2
    - joint_name_3
    - joint_name_4
    - joint_name_5
    - joint_name_6

    stiffness:  # w.r.t. compliance_ref_link
        trans_x: 1000
        trans_y: 1000
        trans_z: 1000
        rot_x: 100
        rot_y: 100
        rot_z: 100

    damping:
        trans: 100
        rot: 100

    pid_gains:
        trans_x: {p: 0.01, i: 0, d: 0}
        trans_y: {p: 0.01, i: 0, d: 0}
        trans_z: {p: 0.01, i: 0, d: 0}
        rot_x: {p: 0.01, i: 0, d: 0}
        rot_y: {p: 0.01, i: 0, d: 0}
        rot_z: {p: 0.01, i: 0, d: 0}
```
