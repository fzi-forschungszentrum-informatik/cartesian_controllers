#/usr/bin/bash
apt -y install libglfw3-dev libglew-dev

cd $HOME
apt install wget
wget https://github.com/deepmind/mujoco/releases/download/3.0.0/mujoco-3.0.0-linux-x86_64.tar.gz
tar -xf mujoco-3.0.0-linux-x86_64.tar.gz
