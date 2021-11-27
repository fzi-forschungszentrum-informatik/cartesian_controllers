![build badge](https://github.com/fzi-forschungszentrum-informatik/cartesian_controllers/actions/workflows/industrial_ci_action.yml/badge.svg)
[![License](https://img.shields.io/badge/License-BSD_3--Clause-blue.svg)](https://opensource.org/licenses/BSD-3-Clause)

# Cartesian Controllers
This package provides libraries with Cartesian ROS controllers.

Check this [short video][roscon19] from ROSCon'19 to get an overview.

It makes the following controller types available for the ROS-control framework:
* for PositionJointInterfaces:
    - position_controllers/CartesianMotionController
    - position_controllers/CartesianComplianceController
    - position_controllers/CartesianForceController

* for VelocityJointInterfaces:
    - velocity_controllers/CartesianMotionController
    - velocity_controllers/CartesianComplianceController
    - velocity_controllers/CartesianForceController

## Installation
Switch into the `src` folder of your current ROS workspace and
```bash
git clone git@github.com:fzi-forschungszentrum-informatik/cartesian_controllers.git
rosdep install --from-paths ./ --ignore-src -y
catkin_make -DCMAKE_BUILD_TYPE=Release
```
Source your workspace again and you are ready to go.

## Getting Started
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
