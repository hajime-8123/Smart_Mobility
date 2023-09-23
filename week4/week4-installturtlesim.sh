#!/bin/sh

echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

export ROS_DOMAIN_ID=0
echo "export ROS_DOMAIN_ID=0" >> ~/.bashrc

export ROS_LOCALHOST_ONLY=1
echo "export ROS_LOCALHOST_ONLY=1" >> ~/.bashrc

echo " Installing turtlesim"
sudo apt install ros-humble-turtlesim

echo " Installing rqt "
sudo apt install ~nros-humble-rqt*
