# Cartesian Skill Controller

This package contains

- a dedicated assembly environment for data creation with teleoperation. No
  robots, just assembly components and interfaces for interaction and data
  recording.

- scripts and algorithmic implementations to learn skills from these data.

## Build and install

1. Install dependencies
   ```bash
   ./.install_mujoco.sh  # required for simulation
   ./.install_python_dependencies.sh  # required for training and serving models in python
   ./.install_libtensorflow_cc.sh  # required for serving models in C++
   ```

2. Switch to the *root* of your ROS2 workspace and build the package (*standalone*) with
   ```bash
   colcon build --packages-select cartesian_skill_controller
   ```

## Launching the simulation
In a sourced terminal, call
```bash
ros2 launch cartesian_skill_controller simulator.launch.py
```

## Teaching in simulation
We use the *3Dconnexion* 3D space mouse that needs additional system dependencies
```bash
sudo apt install libspnav-dev spacenavd screen
```
and these Python dependencies
```bash
pip3 install --user numpy numpy-quaternion
```
We also need a custom version of the `joystick_drivers` (one that correctly reports the button events).
Until pending PRs are merged upstream there, you'll find the package [here](https://github.com/stefanscherzinger/joystick_drivers).
Navigate into your `src` folder in your workspace,
```bash
git clone -b ros2 https://github.com/stefanscherzinger/joystick_drivers.git
```
and build the workspace as usual.

Now launch the teach device with
```bash
ros2 launch cartesian_skill_controller teach_device.launch.py
```
