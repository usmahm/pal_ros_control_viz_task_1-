# ROS 2 Control Ecosystem Visualization - Task 1: ROS 2 Sensor Controller

[Screencast from 08-04-2025 16_59_28.webm](https://github.com/user-attachments/assets/1155ca00-02c5-4ec9-864b-88d1f8faf61b)


This repository contains a ROS 2-based system designed for PAL Robotics GSoC qualification task. The system demonstrates the following functionalities:
1. A publisher node that generates integer sensor data.
2. A client node that subscribes to the sensor data and conditionally sends service requests.
3. A server node that processes the service requests.

This README provides instructions to set up, build, and run the system, along with an explanation of how it works.

---

## How It Works

The system is implemented in both C++ and Python to showcase versatility. The key components are:

![control-viz-1 drawio](https://github.com/user-attachments/assets/bc22f4da-ac05-420c-8fb7-5d55569eebb3)

### Publisher Nodes (`data_publisher.cpp` / `data_publisher.py`)
- Publish an integer message to the `sensor_data` topic at a fixed rate, every 2 seconds.

### Service Server Nodes (`sensor_controller_server.cpp` / `sensor_controller_server.py`)
- Advertise a service (`command_service`) that accepts a boolean request.
- On receiving a request, log the value and respond with `success=true`.

### Client Nodes (`sensor_controller_client.cpp` / `sensor_controller_client.py`)
- Subscribe to the `sensor_data` topic.
- If the incoming message is even (or satisfies a certain condition), send `true` to the service. Otherwise, send `false`.
- Optionally subscribe to the `custom_data` topic (via `interfaces/msg/CustomMessage`) to demonstrate how custom messages can be integrated.

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
