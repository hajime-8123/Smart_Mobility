
# ROS 2 Image Processor Action Server and Client

![ROS 2](https://img.shields.io/badge/ROS%202-Dashing-blue)
![License](https://img.shields.io/badge/License-Apache%202.0-blue)

## Application Description

This ROS 2 package demonstrates the use of actions by implementing a simple Image Processor Action Server and Client. The server receives an image, processes it, and sends back the processed image to the client.

**Key Features:**
- Action server to process images.
- Action client to request image processing.
- Demonstrates ROS 2 action usage.

## Interaction Diagram


![image](https://github.com/hajime-8123/Smart_Mobility/assets/117512935/48e6d634-7874-4e5c-a0e1-097f7d5bdc70)


## Project Overview

The project consists of an action server and an action client. The server processes images using a custom algorithm, while the client sends image processing requests to the server.

### Prerequisites

- ROS 2 Dashing or later
- CMake
- A working ROS 2 installation

### Installation

1. Clone this repository into your ROS 2 workspace.

```bash
git clone https://github.com/your-username/image_processor.git
```

2. Build the package.

```bash
cd /path/to/your/workspace
colcon build --symlink-install
```

### Server

1. Run the Image Processor Server.

```bash
ros2 run image_processor image_processor_server
```

The server will start listening for image processing requests.

### Client

1. Run the Image Processor Client.

```bash
ros2 run image_processor image_processor_client
```

The client will send an image processing request to the server. Customize the request in `image_processor_client.cpp`.

## Customization

You can customize the image processing logic in `image_processor_server.cpp`. Modify the `handle_accepted` function to implement your specific image processing algorithm.

## License

This project is open source and available under the [Apache License 2.0](LICENSE).

## Acknowledgments

- Thanks to the ROS 2 community for their contributions to the ROS ecosystem.

---

Feel free to fork this repository and adapt it to your specific needs. Enjoy ROS 2 development!
