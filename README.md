# RackKI Learning

This package shall contain everything that's related to *learning* in the RackKI project.

Next points that will be addressed:
- A dedicated assembly environment for data creation with teleoperation. No
  robots, just assembly components and interfaces for interaction and data
  recording.
- Scripts and algorithmic implementations to learn skills from these data with
  a special focus on *Offline RL*.

## Build and install
These steps are analog to the `rackki_simulation` package.
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
   wget https://github.com/deepmind/mujoco/releases/download/2.3.1/mujoco-2.3.1-linux-x86_64.tar.gz
   tar -xf mujoco-2.3.1-linux-x86_64.tar.gz
   ```

3. Switch to the *root* of your ROS2 workspace and build the package (*standalone*) with
   ```bash
   colcon build --cmake-args "-DMUJOCO_DIR=$HOME/mujoco-2.3.1"  --packages-select rackki_learning
   ```

## Getting started
In a sourced terminal, call
```bash
ros2 launch rackki_learning simulator.launch.py
```
