## Cartesian Controllers
This package is a library for Cartesian ros controllers.

It provides:
* for robot position interfaces:
    - position_controllers/CartesianMotionController
    - position_controllers/CartesianComplianceController
    - position_controllers/CartesianForceController

* for robot velocity interfaces:
    - velocity_interface/CartesianMotionController
    - velocity_interface/CartesianComplianceController
    - velocity_interface/CartesianForceController

The *motion* and *compliance* controllers from the list above can be connected to a special *adapter* for testing:
* for tf controlled robots
    - cartesian_controllers/JointControllerAdapter

This adapter-controller basically transforms the joint trajectory commands from
a connected joint-based controller to tf-based pose commands for Cartesian
controllers. It only works for controllers from the *position_controllers* family, e.g. *position_controllers/JointTrajectoryController*.


## Features
A special focus of the implementations is the usage of simplified, fast Forward Dynamics computations to circumvent the Inverse Kinematics problem.
As a consequence of this, all controllers can handle robot chains with *1* to *n* joints.
Note, however, that using less than *6* joints results in limited Cartesian responsiveness.
For those systems, the controllers will try to get as close to the Cartesian target as possible, and as best as the robot kinematics allows for it.

With robot chains containing more than *6* joints, the controllers will move the additional_elbows in an energy optimized manner, such that the overall kinetic energy is minimized.

As a further benefit of using Forward Dynamics all controllers can overcome singularities and maintain stability in those configurations.
They will *never* issue configuration changes.

## Limitations
Forward Dynamics turns the search for a feasible mapping of Cartesian input to joint space into a control problem.
The solutions are found iteratively, whereas the system *leaps* forward in virtual time steps until reaching its target.
This means the controllers from this package *do not exactly reproduce Cartesian paths*.
They will reach the target, however, with high precision, if the kinematics allows for it.
The responsiveness of the controllers and with it the path accuracy can be tweaked with the control gains of each controller.
