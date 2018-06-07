## Joint to Cartesian Controller
This adapter-controller basically transforms the joint trajectory commands from
a connected joint-based controller to target pose commands for Cartesian
controllers. It only works for controllers from the *position_controllers* family, e.g. *position_controllers/JointTrajectoryController*.

## Example ##
Below is an example entry for a controller specific configuration using the ur10 robot.
```yaml
my_joint_to_cartesian_controller:
    type: "cartesian_controllers/JointControllerAdapter"
    end_effector_link: "tool0"
    robot_base_link: "base_link"
    target_frame_topic: "target_frame"
    joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint

    joint_limits:
      shoulder_pan_joint:
        has_velocity_limits: true
        max_velocity: 3.15
        has_acceleration_limits: true
        max_acceleration: 10.0
      shoulder_lift_joint:
        has_velocity_limits: true
        max_velocity: 3.15
        has_acceleration_limits: true
        max_acceleration: 10.0
      elbow_joint:
        has_velocity_limits: true
        max_velocity: 3.15
        has_acceleration_limits: true
        max_acceleration: 10.0
      wrist_1_joint:
        has_velocity_limits: true
        max_velocity: 3.2
        has_acceleration_limits: true
        max_acceleration: 10.0
      wrist_2_joint:
        has_velocity_limits: true
        max_velocity: 3.2
        has_acceleration_limits: true
        max_acceleration: 10.0
      wrist_3_joint:
        has_velocity_limits: true
        max_velocity: 3.2
        has_acceleration_limits: true
        max_acceleration: 10.0
```
Joint limits are necessary, because this adapter implements an internal hardware interface and a controller manager to manage connected controllers.
To adapt a joint-based controller to this adapter, you must specify and load the controller to the adapters namespace:
```yaml
my_joint_to_cartesian_controller/joint_trajectory_controller:
    type: "position_controllers/JointTrajectoryController"
    joints:
    - shoulder_pan_joint
    - shoulder_lift_joint
    - elbow_joint
    - wrist_1_joint
    - wrist_2_joint
    - wrist_3_joint
```
And in the launch file for the adapter:
```xml
        <node name="controller_spawner" pkg="controller_manager" type="spawner" args="--stopped my_joint_to_cartesian_controller" />

```
And for the connected joint-based controller:
```xml
        <group ns="my_joint_to_cartesian_controller" >
                <node name="controller_spawner" pkg="controller_manager" type="spawner" args="joint_trajectory_controller" />
        </group>
```
Note the usage of the proper namespace! It is useful to start this joint-based controller on loading, so that the adapter starts publishing valid poses upon activation.
