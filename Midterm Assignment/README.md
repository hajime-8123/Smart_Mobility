# Dynamic Fleet Management Project

## Project Overview

This project aims to implement a dynamic fleet management system using ROS2 (Robot Operating System 2) in Python. The system efficiently allocates and routes vehicles for smart mobility services. The project includes the following components:

1. **ROS2 Package**: A ROS2 package with a meaningful name to encapsulate the project's functionalities.

2. **Launch File**: A launch file to start the ROS2 node for the Action Server.

## Project Tasks

### Task 1: Define ROS2 Action

Create a ROS2 Action file named `fleet_management.action` that defines the Action Goal, Result, and Feedback messages. The Action Goal should include an integer field for the fleet size, the Action Result should include an array of strings for vehicle routes, and the Action Feedback should include a float for the completion percentage.

### Task 2: Implement the Action Server

Create a Python script named `fleet_management_server.py` that implements the Action Server. The server receives fleet size requests, performs fleet management logic (e.g., allocation and routing), and returns the calculated routes as the Action Result. Ensure proper error handling and logging.

### Task 3: Implement the Action Client CLI

Develop a Python script named `fleet_management_client_cli.py` that acts as the Action Client. This client allows users to request fleet management tasks by specifying the fleet size. It sends the request to the server, receives the routes in response, and displays them to the user.

### Task 4: Create a Professional CLI

Design a professional Command Line Interface (CLI) using the `click` library. The CLI provides an option for users to allocate and route vehicles by specifying the fleet size. It internally calls the Action Client CLI to perform the task.

### Task 5: Testing with Scenarios

Create two detailed scenarios for testing the application:

#### Scenario 1: City Bus Service

- **Goal Description**: Allocate and route city buses for a public transportation service in a medium-sized city.
- **Fleet Size**: 8 buses
- **Expected Output (Vehicle Routes)**: Sample routes for the 8 buses, covering major residential areas and transit hubs efficiently.

#### Scenario 2: Food Delivery Service

- **Goal Description**: Allocate and route delivery drivers for a food delivery service.
- **Fleet Size**: 15 delivery drivers
- **Expected Output (Vehicle Routes)**: Sample routes for the 15 drivers to pick up food orders from restaurants and deliver them to customers.

## Project Execution

To execute the project components:

1. Start the ROS2 Core: Run `ros2 run your_package_name ros2_launch_package` in a terminal to start the ROS2 core.

2. Start the Action Server: Execute the Action Server using a launch file with `ros2 launch your_package_name start_fleet_management.launch.py`.

3. Run the Action Client CLI: Send requests to the Action Server with `ros2 run your_package_name fleet_management_client_cli.py --fleet-size [desired_fleet_size]`.

4. Execute the Professional CLI for Scenarios: Use the Professional CLI scripts, such as `ros2 run your_package_name fleet_management_cli.py --fleet-size [desired_fleet_size]`, to initiate the predefined scenarios.

Make sure to replace `'your_package_name'` with the actual name of your ROS2 package and adjust fleet sizes and scenario settings according to your testing needs.

This project allows you to develop and test a dynamic fleet management system for various mobility services.
