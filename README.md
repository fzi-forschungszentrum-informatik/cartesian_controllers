# Cartesian Controllers
This package provides libraries with Cartesian ROS controllers.

It makes the following controller types available for the ROS-control framework:
* for PositionJointInterfaces:
    - position_controllers/CartesianMotionController
    - position_controllers/CartesianComplianceController
    - position_controllers/CartesianForceController

* for VelocityJointInterfaces:
    - velocity_interface/CartesianMotionController
    - velocity_interface/CartesianComplianceController
    - velocity_interface/CartesianForceController

## Getting Started
**Please see the README.md for each controller.**

## Features
All controllers rely on fast forward dynamics computations to circumvent the Inverse Kinematics problem.
This dynamics-based solver is the same for all controller types and provides a consistent interface for configuration.

All controllers can handle robot chains with *1* to *n* joints.
Note, however, that using less than *6* joints results in limited Cartesian responsiveness.
For those systems, the controllers will try to get as close to minimizing the
Cartesian error as possible, and as best as the robot kinematics allows for it.

With robot chains containing more than *6* joints, the controllers will move
the additional elbows in an energy optimized manner.

## Configuration
### Controller gains
Forward dynamics turns the search for a feasible mapping of Cartesian input to joint space into a control problem.
The solutions are found iteratively, whereby the system *leaps* forward in virtual time steps to reduce the Cartesian error.
What this error is depends on the controller type used.
For all controllers, a set of six PID gains (one for each Cartesian dimension)
allows to tweak the responsiveness of the system with respect to this error.
A good practice is to set the p gains of rotation approximately one order of magnitude lower than the translational p gains.
The derivative gains are usually not required.
Also don't use the integral gain. The control plant has an integral part already due to mapping Cartesian input to joint accelerations.

Each controller README.md provides a set of meaningful default gains.
Try starting with these values and continuously adapt those to your special use case.

### Solver parameters
The common solver has two parameters:
* iterations: The number of forward simulated steps for each control cycle.
  Increasing this number will give the solver more iterations to decrease the
  error. However, this mostly makes sense for the CartesianMotionController,
  where increasing means switching from a smoothing controller to a fast and
  exact inverse kinematics solver.

* error_scale: An additional multiplicative factor that uniformly scales the
  6-dimensional PD controlled error (both translation and rotation alike).
  Use this parameter to find the right range
  for your PD gains. It's handy to use the slider in dynamic reconfigure for this.
