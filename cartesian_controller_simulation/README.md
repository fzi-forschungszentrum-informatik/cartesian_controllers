# Cartesian Controller Simulation
This package provides a simulated robot for controller development and testing.

## Physics engine and rationales
We build the simulator on Todorov's [MuJoCo](https://mujoco.org/) physics engine, which
has been acquired and open-sourced by Google [here](https://github.com/deepmind/mujoco).
This gives us a strong environment to realistically test control and contact phenomena with minimal dependencies.


## Build and install
We use MuJoCo in [headless mode](https://mujoco.readthedocs.io/en/latest/programming.html?highlight=headless#using-opengl)
and don't need OpenGL-related dependencies.

1. Download MuJoCo's pre-built [library package](https://github.com/deepmind/mujoco/releases/) and extract that somewhere.
It's ready-to-use and we will just point to it during the build.
   ```bash
   cd $HOME
   wget https://github.com/deepmind/mujoco/releases/download/3.0.0/mujoco-3.0.0-linux-x86_64.tar.gz
   tar -xf mujoco-3.0.0-linux-x86_64.tar.gz
   ```

3. Switch to the *root* of your ROS2 workspace and build the package (*standalone*) with
   ```bash
   colcon build --cmake-args "-DMUJOCO_DIR=$HOME/mujoco-3.0.0" --packages-select cartesian_controller_simulation
   ```


## Getting started
In a sourced terminal, run
```bash
export LC_NUMERIC="en_US.UTF-8"
ros2 launch cartesian_controller_simulation simulation.launch.py
```
The export might not be necessary on your system.
It fixes an eventual *locals* problem and makes sure that you see the robot visualization correctly in RViz.
The *launch* part will start a simulated world with a generic robot model.
You can call
```bash
ros2 control list_controllers
```
to get a list of controllers currently managed by the `controller_manager`.
All of them can be activated and tested in the simulator.
In contrast to `ROS1`, these controllers are nodes and you can also see them with `ros2 node list`.


## Hardware interfaces for controllers
Exposed interfaces per joint:

- `command_interfaces`: position, velocity, stiffness, damping
- `state_interfaces`: position, velocity
