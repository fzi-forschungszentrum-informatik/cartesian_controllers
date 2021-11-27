![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_action.yml/badge.svg)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Cartesian Controllers
This package provides a set of Cartesian `motion`, `force` and `compliance controllers` for the `ROS-control` framework.
The controllers are meant for `joint_position` and `joint_velocity` interfaces on the manipulators.
As a unique selling point, they use fast forward dynamics simulations of
virtually conditioned twins of the real setup as a solver for the inverse kinematics problem.
They are designed to trade smooth and stable behavior for accuracy where
appropriate, and behave physically plausible for targets outside the robots reach.
The package is for users who require interfaces to direct task space control
without the need for collision checking.
See [this talk at ROSCon'19](https://vimeo.com/378682968) and [the
slides](https://roscon.ros.org/2019/talks/roscon2019_cartesiancontrollers.pdf)
to get an overview.

## Why this package?
Users may refer to `MoveIt` for end-effector motion planning, but 
integrating a full planning stack is often unnecessary for simple applications.
Additionally, there are a lot of use cases where direct control in task space is mandatory:
Dynamic following of target poses, such as **visual servoing**, **teleoperation**, **Cartesian teaching,** or
any form of **closed loop control with external sensors** for physical interactions with environments.
This package provides such a controller suite for the [ROS-control](http://wiki.ros.org/ros_control) framework.

## Installation
Switch into the `src` folder of your current ROS workspace and
```bash
git clone git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```
Source your workspace again and you are ready to go.

## Getting started
In a sourced terminal, call `roslaunch cartesian_controller_examples
examples.launch `. This will start a simulated world in which you can inspect
and try things. Here are some quick tutorials with further details:
- [Solver details](resources/doc/Solver_details.md)
- [Cartesian motion controller](cartesian_motion_controller/README.md)
- [Cartesian force controller](cartesian_force_controller/README.md)
- [Cartesian compliance controller](cartesian_compliance_controller/README.md)
- [Cartesian controller handles](cartesian_controller_handles/README.md)

## Further reading
If you are interested in more details, check out [the paper][paper1] on the initial idea of the compliance controller,
and [the paper][paper2] on the recent implementation of the IK solver for motion control.

[paper1]: https://ieeexplore.ieee.org/document/8206325 "Forward Dynamics Compliance Control (FDCC)"
[paper2]: https://arxiv.org/pdf/1908.06252.pdf "Inverse Kinematics with Forward Dynamics for Sampled Motion Tracking"
[roscon19]: https://vimeo.com/378682968
