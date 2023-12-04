# Smart Mobility Lab - TurtleBot Elderly Assistant Robot Project

## Overview
This project aims to address the challenges associated with the aging population by implementing Elderly Assistant Robot system for elderly people using TurtleBot (Burger). The system provides companionship, support, and healthcare assistance to enhance the quality of life for the elderly.

## ROS 2 Foxy Installation
Make sure you have ROS 2 Foxy installed on your system. If not, follow the installation instructions at [ROS 2 Foxy Installation Guide](https://docs.ros.org/en/foxy/Installation.html).

## Gazebo Installation
Install Gazebo to enable simulation of the TurtleBot. Follow the installation instructions at [Gazebo Installation Guide](http://gazebosim.org/tutorials?tut=install_ubuntu).

## ROS 2 Workspace Setup
1. Clone this repository:
    ```bash
    git clone <repository_url>
    ```

2. Navigate to the workspace directory:
    ```bash
    cd ros2_ws
    ```

3. Build the workspace:
    ```bash
    colcon build
    ```

4. Source the workspace setup file:
    ```bash
    source install/setup.bash
    ```

## Package Structure
The project is organized into the following ROS 2 package:

### logging_monitoring
This package contains nodes for logging and monitoring various aspects of the HCI system for the elderly. Each node focuses on a specific functionality, such as battery level monitoring, emotion recognition, gesture recognition, etc.

### Node Descriptions
1. **battery_power_node.py**: Monitors the battery level of the TurtleBot.
2. **emotion_recognition.py**: Node for emotion recognition (provide additional details).
3. **gesture_recognition_monitor_node.py**: Node for gesture recognition monitoring (provide additional details).
4. **logging_node.py**: General logging node capturing system information.
5. **obstacle_avoidance.py**: Node for obstacle avoidance (provide additional details).
6. **sensor_data_simulator.py**: Node for simulating sensor data (provide additional details).
7. **voice_recognition_monitor_node.py**: Node for voice recognition monitoring (provide additional details).

## Launch Files
Launch files are available for each node to facilitate easy startup. The launch files are located in the `launch` directory.

### Example Launch Command
Replace `<node_launch_file>` with the desired node launch file.
```bash
ros2 launch logging_monitoring <node_launch_file>
```
## Topic Exploration
Explore the available ROS 2 topics using the following command:
```bash
ros2 topic list
```
## Contributing
Feel free to contribute to the project by opening issues or creating pull requests. Your input is valuable in improving the functionality and reliability of the HCI system.
