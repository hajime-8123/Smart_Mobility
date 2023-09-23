#!/bin/sh

# Installation process

locale  # check for UTF-8
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings

# Enabling Ubuntu Universe
sudo apt install software-properties-common
sudo add-apt-repository universe

# Adding the ROS 2 GPG key with apt
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Adding the repository to your sources list
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Updating and Upgrading apt repository caches and system.
sudo apt update & sudo apt upgrade

# Desktop Install.
sudo apt install ros-humble-desktop

# C++ testing code
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# testing python code
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_py listener
