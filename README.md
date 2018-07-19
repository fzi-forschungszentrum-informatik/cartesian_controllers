# Cartesian Controllers
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

For the *motion* and *compliance* controllers from the list above there are two special controllers for an easy generation of the required target poses:
* for PoseStamped controlled robots
    - cartesian_controllers/JointControllerAdapter
    - cartesian_controllers/MotionControlHandle

The first one is an adapter-controller and basically transforms the joint
trajectory commands from a connected joint-based controller to target pose
commands. This means you could execute joint-based trajectories with the
cartesian_controllers. The target poses are generated on-the-fly.
It only works for controllers from the *position_controllers* family, e.g. *position_controllers/JointTrajectoryController*.

The second controller is a click-and-drop handle for RViz that publishes the current pose of the according interactive marker as a target pose.

**Please check the according README.md for each controller type**


## Features
A special focus of the implementations is the usage of simplified, fast Forward Dynamics computations to circumvent the Inverse Kinematics problem.
As a consequence of this, all controllers can handle robot chains with *1* to *n* joints.
Note, however, that using less than *6* joints results in limited Cartesian responsiveness.
For those systems, the controllers will try to get as close to the Cartesian target as possible, and as best as the robot kinematics allows for it.

With robot chains containing more than *6* joints, the controllers will move the additional_elbows in an energy optimized manner, such that the overall kinetic energy is minimized.

As a further benefit of using Forward Dynamics all controllers can overcome singularities and maintain stability in those configurations.
They will *never* issue configuration changes.

All controllers interpolate the given target poses. The amount of *how much* they interpolate can be set through the responsiveness. Check the section on *damping* for that.
As an example, the *CartesianMotionController* can be given target poses,
that pop up somewhere with a relatively low frequency. Depending on how
responsive the system is, this can result in anything in between fast jumps or
smooth, directed motion.

## Limitations
Forward Dynamics turns the search for a feasible mapping of Cartesian input to joint space into a control problem.
The solutions are found iteratively, whereby the system *leaps* forward in virtual time steps until reaching its target.
This means the controllers from this package *do not exactly reproduce Cartesian paths*.
They will reach the target, however, with high precision, if the kinematics allows for it.
The responsiveness of the controllers and with it the path accuracy can be tweaked with a special kind of damping and the control gains of each controller.

## Controller gains
The controller gains from each README.md represent sane default values for each controller type. If you don't have good reasons not to, then use those defaults.
The responsiveness, and with it contact stability, should be adjusted via the solver specific *mass* and *inertia* parameters.
Using an iterative integration method in the forward dynamics solver enables the system to reach its targets without overshooting.
This works, because the internal model leaps forward step-wise, based on a Cartesian offset, without accumulating velocity.
This means you don't have to add derivative gains to make the system stable, nor integral gains to achieve steady state accuracy.

## Damping
All controllers personalize two solver parameters: the *mass* and the *inertia* of the apparent end-effector.
They don't implement damping in the physical sense, as in *oppose motion with a force*, but they achieve the same effect and have the advantage to scale the system responsiveness and to *slow it down* with only few parameters.
Since the system never overshoots, this provides a convenient interface to adjust contact stability and path accuracy.

## Path accuracy
The path accuracy gets better the smaller both *mass* and *inertia* are set.
For *CartesianMotionControllers* this can be set to the minimum, as mentioned in the according *README.md*.
Note, however, that the maximal joint velocities of the robot usually represent
the limiting factor. If you notice that decreasing *mass* and *inertia* does
not make the robot move as fast as you would like, then probably the joint
velocity limits are already reached for this motion. Reaching these limits on
individual joints also leads to a distortion of the Cartesian path. Only if you
are sure that your robot can move faster, then also start adjusting the
*proportional gains* of the controller.

The *CartesianForceControllers* and the *CartesianComplianceControllers* are designed for contact-dominated tasks. During these tasks, path accuracy is traded for contact stability, which is increased through increasing *mass* and *inertia*. It might be necessary to increase those parameters quite a bit, if the environments are very stiff.
