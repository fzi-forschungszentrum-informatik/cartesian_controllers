# Cartesian Force Controller
This package provides a controller for end effector force control.

## Getting Started
1) In a sourced terminal, run
```bash
roslaunch cartesian_controller_examples examples.launch
```
2) In another sourced terminal, open rqt and navigate to the *Controller Manager* plugin under *Robot Tools*.
Select */controller_manager* as namespace and activate *my_cartesian_force_controller*.

3) Publish a geometry_msgs/WrenchStamped to */my_cartesian_force_controller/target_wrench* with force x = 2 and watch the robot move.

4) In rqt open the *Dynamic Reconfigure* plugin under *Configuration*. Play a
little with the parameters of *my_cartesian_force_controller* (e.g. solver/error_scale) and observe the
changes in RViz with publishing different values along various axes.
Also try forcing the end effector through singularities and get a feeling how the controller behaves.

## Example Configuration
Below is an example entry for a controller specific configuration. Also see cartesian_controller_examples/config/example_controllers.yaml for further tips.
```yaml
my_cartesian_force_controller:
    type: "position_controllers/CartesianForceController"
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    ft_sensor_ref_link: "sensor_link"
    joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6

    pd_gains:
        trans_x: {p: 0.05}
        trans_y: {p: 0.05}
        trans_z: {p: 0.05}
        rot_x: {p: 0.01}
        rot_y: {p: 0.01}
        rot_z: {p: 0.01}

```

The controller configuration must be loaded to the ros parameter server and is
accessed by the controller manager when looking for configuration for the
loaded controller *my_cartesian_force_controller*.

## Tips
Note that the controller does not strictly move only in the commanded direction.
Sometimes there's a small drift in other axes. This is a feature of the forward dynamics solver.
In fact, there is no error reduction on axes orthogonal to the target wrench.
The benefit is that the controller finds approximate solutions near singular configurations.
