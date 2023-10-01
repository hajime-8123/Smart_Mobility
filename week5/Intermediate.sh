#!/bin/bash

# Install python3-rosdep
sudo apt-get install python3-rosdep

# Install rosdep using pip
pip install rosdep

# Initialize rosdep
sudo rosdep init

# Update rosdep database
rosdep update

# Install ROS dependencies from paths (assumes 'src' directory)
rosdep install --from-paths src -y --ignore-src

# Create the workspace and navigate to the source directory
mkdir -p ros2_ws/src
cd ros2_ws/src

# Create the action package
ros2 pkg create action_tutorials_interfaces
cd action_tutorials_interfaces
mkdir action

# Add the action definition to CMakeLists.txt
echo 'find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/Fibonacci.action"
)

<buildtool_depend>rosidl_default_generators</buildtool_depend>
<depend>action_msgs</depend>
<member_of_group>rosidl_interface_packages</member_of_group>' > CMakeLists.txt

# Change back to the root of the workspace
cd ~/ros2_ws

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash

# Check the action definition
ros2 interface show action_tutorials_interfaces/action/Fibonacci

# Change directory to the ROS2 workspace source folder
cd ~/ros2_ws/src

# Create a ROS2 package
ros2 pkg create --dependencies action_tutorials_interfaces rclcpp rclcpp_action rclcpp_components -- action_tutorials_cpp

# Build the ROS2 packages using colcon
colcon build

# Run the Fibonacci action server
ros2 run action_tutorials_cpp fibonacci_action_server

# Execute the Python script for the Fibonacci action server
python3 fibonacci_action_server.py

# Send a goal to the Fibonacci action server
ros2 action send_goal fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# Send a goal with feedback to the Fibonacci action server
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{order: 5}"

# Execute the Python script for the Fibonacci action server again
python3 fibonacci_action_server.py

# List ROS2 component types
ros2 component types

# Run the component container
ros2 run rclcpp_components component_container

# List loaded components
ros2 component list

# Load composition components
ros2 component load /ComponentManager composition composition::Talker

# Run the manual composition
ros2 run composition manual_composition

# Load composition components with dynamic libraries
ros2 run composition dlopen_composition `ros2 pkg prefix composition`/lib/libtalker_component.so `ros2 pkg prefix composition`/lib/liblistener_component.so

# Launch the composition demo
ros2 launch composition composition_demo.launch.py

# Unload composition components
ros2 component unload /ComponentManager 1 2

# Run component container with custom node name and namespace
ros2 run rclcpp_components component_container --ros-args -r __node:=MyContainer -r __ns:=/ns

# Load composition components with custom namespace
ros2 component load /ns/MyContainer composition composition::Listener

# Load composition components with custom node name and namespace
ros2 component load /ComponentManager composition composition::Talker --node-name talker2
ros2 component load /ComponentManager composition composition::Talker --node-namespace /ns
ros2 component load /ComponentManager composition composition::Talker --node-name talker3 --node-namespace /ns2

# Load image_tools component with parameters
ros2 component load /ComponentManager image_tools image_tools::Cam2Image -p burger_mode:=true

# Load composition components with intra-process communication enabled
ros2 component load /ComponentManager composition composition::Talker -e use_intra_process_comms:=true

# Create a ROS2 package with dependencies
ros2 pkg create --build-type ament_cmake cpp_parameter_event_handler --dependencies rclcpp

# Install dependencies using rosdep
rosdep install -i --from-path src --rosdistro $ROS_DISTRO -y

# Build the specified package
colcon build --packages-select cpp_parameter_event_handler

# Source the setup.bash file
. install/setup.bash

# Run the parameter_event_handler node
ros2 run cpp_parameter_event_handler parameter_event_handler

# Set a ROS2 parameter
ros2 param set node_with_parameters an_int_param 43

# Rebuild the specified package
colcon build --packages-select cpp_parameter_event_handler

# Run the parameter_event_handler node again
ros2 run cpp_parameter_event_handler parameter_event_handler

# Run the demo_nodes_cpp parameter_blackboard node
ros2 run demo_nodes_cpp parameter_blackboard

# Set a parameter for the parameter_blackboard node
ros2 param set parameter_blackboard a_double_param 3.45

# Create and navigate to the 'launch' directory
mkdir -p launch
cd launch

# Launch turtlesim_mimic_launch.py
ros2 launch turtlesim_mimic_launch.py &

# Wait for a moment to allow the launch to complete
sleep 5

# Publish a Twist message to control the turtle
ros2 topic pub -r 1 /turtlesim1/turtle1/cmd_vel geometry_msgs/msg/Twist "{linear: {x: 2.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.8}}" &

# Open rqt_graph
rqt_graph &

# Create and navigate to the 'launch_ws/src' directory
mkdir -p launch_ws/src
cd launch_ws/src

# Create a ROS2 package for py_launch_example
ros2 pkg create py_launch_example --build-type ament_python

# Launch my_script_launch.py
ros2 launch py_launch_example my_script_launch.py &

# Create a ROS2 package for launch_tutorial
ros2 pkg create launch_tutorial --build-type ament_python

# Create the 'launch' directory inside launch_tutorial
mkdir -p launch_tutorial/launch

# Launch example_main.launch.py
ros2 launch launch_tutorial example_main.launch.py &

# Launch example_substitutions.launch.py with custom arguments
ros2 launch launch_tutorial example_substitutions.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200 &

# Launch example_event_handlers.launch.py with custom arguments
ros2 launch launch_tutorial example_event_handlers.launch.py turtlesim_ns:='turtlesim3' use_provided_red:='True' new_background_r:=200 &

# Run turtle_teleop_key
ros2 run turtlesim turtle_teleop_key

