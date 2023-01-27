![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_melodic_action.yml/badge.svg)
![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_noetic_action.yml/badge.svg)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

---

**ROS2** support is [here](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2).

---


# Cartesian Controllers
This package provides a set of Cartesian `motion`, `force` and `compliance controllers` for the `ros_control` framework.
The controllers are meant for `joint position` and `joint velocity` interfaces on the manipulators.
As a unique selling point, they use fast forward dynamics simulations of
virtually conditioned twins of the real setup as a solver for the inverse kinematics problem.
Integrating from joint accelerations to joint velocities and joint positions
gives them a delay-free, noise suppressing, and an inherently more stable contact behavior than conventional
*Admittance* controllers.
The controllers from this package are designed to trade smooth and stable behavior for accuracy where
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
dynamic following of target poses, such as **visual servoing**, **teleoperation**, **Cartesian teaching,** or
any form of **closed loop control with external sensors** for physical interactions with environments, such as **Machine Learning** applications.
This package provides such a controller suite for the [ros_control](http://wiki.ros.org/ros_control) framework.

## Installation
Switch into the `src` folder of your current ROS workspace and
```bash
git clone git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
cd ..
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
- [Teleoperation](cartesian_controller_utilities/README.md)

## Citation and further reading
If you use the *cartesian_controllers* in your research projects, please
consider citing our initial idea of the forward dynamics-based control
approach ([Paper](https://ieeexplore.ieee.org/document/8206325)):
```bibtex
@InProceedings{FDCC,
  Title                    = {Forward Dynamics Compliance Control (FDCC): A new approach to cartesian compliance for robotic manipulators},
  Author                   = {S. Scherzinger and A. Roennau and R. Dillmann},
  Booktitle                = {IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
  Year                     = {2017},
  Pages                    = {4568-4575},
  Doi                      = {10.1109/IROS.2017.8206325}
}

```

If you are interested in more details, have a look at
- *Inverse Kinematics with Forward Dynamics Solvers for Sampled Motion Tracking* ([Paper](https://arxiv.org/pdf/1908.06252.pdf))
- *Virtual Forward Dynamics Models for Cartesian Robot Control* ([Paper](https://arxiv.org/pdf/2009.11888.pdf))
- *Contact Skill Imitation Learning for Robot-Independent Assembly Programming* ([Paper](https://arxiv.org/pdf/1908.06272.pdf))
- *Human-Inspired Compliant Controllers for Robotic Assembly* ([PhD Thesis](https://publikationen.bibliothek.kit.edu/1000139834), especially Chapter 4)

