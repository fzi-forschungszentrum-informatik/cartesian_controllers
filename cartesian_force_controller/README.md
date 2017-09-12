## Cartesian Force Controller ##

This package provides a ros-controller which implements free-floating force control on a set of joints.
Like all Cartesian controllers, this controller has an internal *damping*, which affects the responsiveness of the solver and can be set with:
```yaml
solver:
    mass: 15
    inertia: 0.05
```

## Example ##
An example controller configuration for the parameter server looks like the following:
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

    solver:
        mass: 15
        inertia: 0.05

    pid_gains:
        trans_x: {p: 0.05, i: 0, d: 0}
        trans_y: {p: 0.05, i: 0, d: 0}
        trans_z: {p: 0.05, i: 0, d: 0}
        rot_x: {p: 0.01, i: 0, d: 0}
        rot_y: {p: 0.01, i: 0, d: 0}
        rot_z: {p: 0.01, i: 0, d: 0}
```
A minimal example can be found in *cartesian_controller_test* of this meta package.
Also check the top-level **README.md** for further information.
