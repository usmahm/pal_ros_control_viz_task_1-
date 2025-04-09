# ROS 2 Sensor Controller (Task 1)

This repository contains a ROS 2-based system designed as part of a GSoC qualification task. The system demonstrates the following functionalities:
1. A publisher node that generates integer sensor data.
2. A client node that subscribes to the sensor data and conditionally sends service requests.
3. A server node that processes the service requests.

This README provides instructions to set up, build, and run the system, along with an explanation of the approach taken to solve the task.

---

## Approach to the Solution

The system is implemented in both C++ and Python to showcase versatility. The key components are:
- **Publisher Node**: Publishes integer data on a topic (`sensor_data`).
- **Client Node**: Subscribes to the topic and sends service requests based on the data (e.g., even/odd logic).
- **Server Node**: Responds to the service requests and logs the results.

---

### Description of Folder Structure

- **`interfaces/`**: Contains a custom message definition (`CustomMessage.msg`) for optional use in the system.
- **`sensor_controller_cpp/`**: C++ implementation of the publisher, client, and server nodes.
- **`sensor_controller_py/`**: Python implementation of the same nodes.

---

## Prerequisites

- ROS 2 Humble, correctly sourced in your shell.
- `colcon` build system.
- C++14/C++17 compiler for C++ nodes.
- Python 3.x for Python scripts.

---

## Building Instructions

1. Clone or copy the repository into your ROS 2 workspace:
   ```bash
   mkdir -p ~/ros2_ws/src
   cd ~/ros2_ws/src
   git clone https://github.com/<your-username>/sensor_controller_task.git
   ```

2. Build the workspace:
   ```bash
   cd ~/ros2_ws
   source /opt/ros/<distro>/setup.bash
   colcon build
   source install/setup.bash
   ```
   Replace `<distro>` with your ROS 2 distribution (e.g., `humble`, `foxy`).

---

## Running the System

### Running the Python Version

1. **Terminal A**: Start the server node:
   ```bash
   ros2 run sensor_controller_py sensor_controller_server
   ```

2. **Terminal B**: Start the publisher node:
   ```bash
   ros2 run sensor_controller_py data_publisher
   ```

3. **Terminal C**: Start the client node:
   ```bash
   ros2 run sensor_controller_py sensor_controller_client
   ```

### Running the C++ Version

1. **Terminal A**: Start the server node:
   ```bash
   ros2 run sensor_controller_cpp sensor_controller_server
   ```

2. **Terminal B**: Start the publisher node:
   ```bash
   ros2 run sensor_controller_cpp data_publisher
   ```

3. **Terminal C**: Start the client node:
   ```bash
   ros2 run sensor_controller_cpp sensor_controller_client
   ```

---

## Logs and Behavior

- The publisher node logs the integers it publishes.
- The client node logs the boolean requests it sends based on the integer data.
- The server node logs the received requests and responds with `success=true`.
