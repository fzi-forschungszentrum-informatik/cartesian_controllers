# RackKI Learning

This package contains

- a dedicated assembly environment for data creation with teleoperation. No
  robots, just assembly components and interfaces for interaction and data
  recording.

- scripts and algorithmic implementations to learn skills from these data.

## Build and install
We use MuJoCo with the [X11 OpenGL setting](https://mujoco.readthedocs.io/en/latest/programming.html#using-opengl) and
need both [GLFW](https://www.glfw.org/) and [GLEW](http://glew.sourceforge.net/).

1. Install them as system dependencies with
   ```bash
   sudo apt-get install libglfw3-dev libglew-dev
   ```
   OpenGL itself should ship with recent Ubuntu OS.

2. Download MuJoCo's most recent [release](https://github.com/deepmind/mujoco/releases/) and extract that somewhere.
It's a ready-to-use, pre-built library package, and we will just point to it during the build.
   ```bash
   cd $HOME
   wget https://github.com/deepmind/mujoco/releases/download/3.0.0/mujoco-3.0.0-linux-x86_64.tar.gz
   tar -xf mujoco-3.0.0-linux-x86_64.tar.gz
   ```

3. Switch to the *root* of your ROS2 workspace and build the package (*standalone*) with
   ```bash
   colcon build --cmake-args "-DMUJOCO_DIR=$HOME/mujoco-3.0.0"  --packages-select rackki_learning
   ```

## Launching the simulation
In a sourced terminal, call
```bash
ros2 launch rackki_learning simulator.launch.py
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
ros2 launch rackki_learning teach_device.launch.py
```
