#!/bin/bash

# First setup discovery server
fastdds discovery --server-id 0 &

# Wait for the discovery server to start
sleep 2

# Set the environment variable for ROS discovery server
export ROS_DISCOVERY_SERVER=127.0.0.1:11811

# Launch listener node
ros2 run demo_nodes_cpp listener --ros-args --remap __node:=listener_discovery_server &

# Launch talker node in a new terminal
gnome-terminal -- bash -c "export ROS_DISCOVERY_SERVER=127.0.0.1:11811; ros2 run demo_nodes_cpp talker --ros-args --remap __node:=talker_discovery_server" &

# Finally, check that everything is running correctly
ros2 run demo_nodes_cpp talker --ros-args --remap __node:=simple_talker

