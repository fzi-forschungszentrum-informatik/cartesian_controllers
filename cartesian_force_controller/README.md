# Cartesian Force Controller

This controller is for end-effector force control in contact with the robot's environment.
It's also a good choice for intuitively steering the robot in transition tasks
(from motion to contact) through teleoperation, or for combining it with the low-level
action space of AI-based policies.

The responsiveness of the behavior can be tweaked by the following parameters:
* The `p` and `d` gains determine the responsiveness in each individual Cartesian axis. The higher these
  values, the faster does the robot *move* in response to either the sensed contact forces or the published target wrench.
  Proportional gains (`p`) are normally sufficient and you'll probably not need
  a `d` gain for most use cases. With a little bit of tweaking, though, they sometimes help to reduce oscillations in stiff contacts.
* The `error_scale` influences the controller's sensitivity uniformly for all axes. It
  post-scales the computed error from the controller gains (by multiplication) and is a little
  easier to adjust with a single parameter. Similar to pure motion control, the idea is
  to first weigh the different Cartesian axes with the controller gains and adjust the
  *overall* responsiveness with this parameter.

In contrast to the `cartesian_motion_controller`, the internal `iterations` parameter has no effect and is identically set to `1`.
Also note that all parameters can be changed at runtime and in realtime.
This means that you could easily adjust them in your use cases with a running robot.
For instance, `error_scale` could start in a higher value when teleoperating
the robot and could be further decreased upon detecting contact to maintain
stability during a guided assembly operation.


## Getting Started
We assume that you have the `cartesian_controller_simulation` package installed.
1) Start the simulation environment as described [here](./../cartesian_controller_simulation/README.md).

2) Now we activate the `cartesian_force_controller`:
   ```bash
   ros2 control switch_controllers --start cartesian_force_controller
   ```

3) Next, open `rqt` in a new, sourced terminal to publish a target wrench and see the robot move:
   ```bash
   rqt
   ```
   Navigate to `Plugins -> Topics -> Message Publisher` and configure the following window:
   ![Target wrench publisher](resources/images/target_wrench_publisher.png)

   Upon checking the checkbox for `/target_wrench`, the robot should move.
   The published force will pull the robot in some direction and you
   can experiment a little with different force-torque components.

4) Change some of the controller's parameters:
   In `rqt`, open the *Dynamic Reconfigure* plugin under *Plugins/Configuration* and
   select the *cartesian_force_controller*. Play a little with the parameters
   (e.g. `solver/error_scale = 0`) and observe the changes of behavior in RViz.

When working with a new robot for the first time, it's good practice to start
with `error_scale = 0` and carefully increment that (e.g. in `rqt`) while
touching and guiding the robot's end-effector by hand.
The controller should follow your lead and behave plausible in each Cartesian dimension.
If that's not the case, there might be some issue with the sensor setup.

## Example Configuration
Below is an example `controller_manager.yaml` for a controller specific configuration. Also see [the simulation config](../cartesian_controller_simulation/config/controller_manager.yaml) for further information.
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

    cartesian_force_controller:
      type: cartesian_force_controller/CartesianForceController

    # More controller instances here
    # ...

cartesian_force_controller:
  ros__parameters:
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

    # Choose: position or velocity.
    command_interfaces:
      - position
        #- velocity

    solver:
        error_scale: 0.5

    pd_gains:
        trans_x: {p: 0.05}
        trans_y: {p: 0.05}
        trans_z: {p: 0.05}
        rot_x: {p: 1.5}
        rot_y: {p: 1.5}
        rot_z: {p: 1.5}


# More controller specifications here
# ...

```

## Additional insights
Note that the controller does not strictly move only in the commanded direction.
Although its behavior is linearized in operational space, there might be small drifts in other axes.
This is a feature of the forward dynamics-based solver.
In fact, there is no error reduction on axes orthogonal to the published target wrench.
The benefit is that the controller finds approximate solutions near singular configurations.
