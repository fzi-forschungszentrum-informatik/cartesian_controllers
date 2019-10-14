# Cartesian Controller Handles
This package provides graphical click-and-drop handles (interactive markers) for
RViz to be used in conjunction with the CartesianMotionControllers.

The markers can be used to control the robot end-effector online, which is
especially handy for testing scenarios and Cartesian trajectory teaching.

## Getting started
The controller handles are implemented as ROS controllers, i.e. you must load and start them as you would with normal controllers.
During operation a controller of type *CartesianMotionController* must be active for that robot to execute your target motion.
For the interactive handle load your configuration to the parameter server, e.g:
``` xml
my_motion_control_handle:
   type: "cartesian_controllers/MotionControlHandle"
   end_effector_link: "tool0"
   robot_base_link: "base_link"
   target_frame_topic: "/my_cartesian_motion_controller/target_frame"
   joints:
    - joint1
    - joint2
    - joint3
    - joint4
    - joint5
    - joint6
```
And spawn the handle accordingly:
```xml
<node name="controller_spawner" pkg="controller_manager" type="spawner" args="--stopped my_motion_control_handle" />
```
Explicitly start it whenever you want to interactively move your robot with RViz.
Note: Make sure that no other controllers are publishing a *target* to your *CartesianMotionController* while using the motion control handle.

## RViz
You must create a visualization in RViz to see and interact with the colored handles. Add *InteractiveMarkers* to your *Displays* menu and point it to the right *Update Topic*.
The interactive handle only gets visualized if your **MotionControlHandle** is running. If you still see no handles, try toggling the *Interactive Markers*'s checkbox.
