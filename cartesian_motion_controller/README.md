## Cartesian Motion Controller ##

This package provides a controller for mapping Cartesian end-effector motion to joint control commands of a set of joints.
The controller tracks a given tf target frame. The responsiveness should be adjusted with the solver specific *mass* and *inertia*, and *not* with the controller gains.

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

    solver:
        mass: 2.0
        inertia: 0.05

    pid_gains:
        trans_x: {p: 10.0, i: 0, d: 0}
        trans_y: {p: 10.0, i: 0, d: 0}
        trans_z: {p: 10.0, i: 0, d: 0}
        rot_x: {p: 1.0, i: 0, d: 0}
        rot_y: {p: 1.0, i: 0, d: 0}
        rot_z: {p: 1.0, i: 0, d: 0}
```

The solver specific entry represents the default configuration, which is active, if nothing is set on the parameter server.
The configuration for maximum responsiveness is (as currently defined in the *dynamic_reconfigure.cfg* in the base package):
```yaml
solver:
    mass: 1.0
    inertia: 0.002
```

The controller configuration must be loaded to the ros parameter server and is accessed by the controller manager when looking for configuration for the loaded controller *my_cartesian_motion_controller*.
A minimal example can be found in *cartesian_controller_test* of this meta package.
Also check the top-level **README.md** for further information.
