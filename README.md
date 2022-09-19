![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_foxy_action.yml/badge.svg)
![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_galactic_action.yml/badge.svg)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

---


This is the **ROS2** version of the `cartesian_controllers`. It's almost feature complete. Checkout the individual controller READMEs and have a look at the [cartesian_controllers_simulation](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/tree/ros2/cartesian_controller_simulation) package for a complete example. 

---


# Cartesian Controllers
This package provides a set of Cartesian `motion`, `force` and `compliance controllers` for the `ROS2-control` framework.
The controllers are meant for `joint position` and `joint velocity` interfaces on the manipulators.
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
dynamic following of target poses, such as **visual servoing**, **teleoperation**, **Cartesian teaching,** or
any form of **closed loop control with external sensors** for physical interactions with environments.
This package provides such a controller suite for the [ROS2-control](https://control.ros.org/master/index.html) framework.

## Installation
Switch into the `src` folder of your current ROS2 workspace and
```bash
git clone -b ros2-devel git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
cd ..
colcon build --packages-skip cartesian_controller_simulation --cmake-args -DCMAKE_BUILD_TYPE=Release
```
This builds the `cartesian_controllers` without its simulation environment.
The simulation is mostly relevant if you are just getting to know the `cartesian_controllers` and want to inspect how things work.
You can install it according to this [readme](cartesian_controller_simulation/README.md).

Now source your workspace again and you are ready to go.

## Getting started
This assumes you have the `cartesian_controller_simulation` package installed.
In a sourced terminal, call
```bash
ros2 launch cartesian_controller_simulation simulation.launch.py
```

This will start a simulated world in which you can inspect
and try things. Here are some quick tutorials with further details:
- [Solver details](resources/doc/Solver_details.md)
- [Cartesian motion controller](cartesian_motion_controller/README.md)
- [Cartesian force controller](cartesian_force_controller/README.md)
- [Cartesian compliance controller](cartesian_compliance_controller/README.md)
- [Cartesian controller handles](cartesian_controller_handles/README.md)

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

