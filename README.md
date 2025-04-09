# ROS 2 Sensor Controller (Task 1)

This repository contains a ROS 2-based system designed as part of a GSoC qualification task. The system demonstrates the following functionalities:
1. A publisher node that generates integer sensor data.
2. A client node that subscribes to the sensor data and conditionally sends service requests.
3. A server node that processes the service requests.
<!-- 4. (Optional) Usage of a custom message defined in the `interfaces/` package. -->

This README provides instructions to set up, build, and run the system, along with an explanation of the approach taken to solve the task.

---

## Approach to the Solution

<!-- The solution is structured to demonstrate modularity and flexibility in ROS 2 development.  -->
The system is implemented in both C++ and Python to showcase versatility. The key components are:
- **Publisher Node**: Publishes integer data on a topic (`sensor_data`).
- **Client Node**: Subscribes to the topic and sends service requests based on the data (e.g., even/odd logic).
- **Server Node**: Responds to the service requests and logs the results.
<!-- - **Custom Message**: An optional feature to demonstrate advanced ROS 2 capabilities. -->

<!-- The implementation ensures clear separation of concerns, making it easy to extend or modify individual components. -->

---

<!-- ## Folder Structure

```bash
.
├── interfaces/
│   ├── CMakeLists.txt
│   └── msg/
│       └── CustomMessage.msg
├── sensor_controller_cpp/
│   ├── CMakeLists.txt
│   ├── package.xml
│   ├── data_publisher.cpp
│   ├── sensor_controller_client.cpp
│   └── sensor_controller_server.cpp
└── sensor_controller_py/
    ├── setup.py
    ├── package.xml
    ├── data_publisher.py
    ├── sensor_controller_client.py
    └── sensor_controller_server.py
``` -->

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

<!-- --- -->

<!-- ## Custom Message Usage

The `interfaces/msg/CustomMessage.msg` defines a custom message with the following fields:
```plaintext
bool is_hardware
string metadata
```

To use the custom message:
1. Build the `interfaces` package first.
2. Publish or subscribe to the `custom_data` topic using the custom message.

### Example:
- **C++**: Include `interfaces/msg/custom_message.hpp` and subscribe to `custom_data`.
- **Python**: Import `interfaces.msg._custom_message` and handle the custom fields in a subscriber. -->

---

## Logs and Behavior

- The publisher node logs the integers it publishes.
- The client node logs the boolean requests it sends based on the integer data.
- The server node logs the received requests and responds with `success=true`.

<!-- This modular design ensures clarity and ease of debugging. -->

<!-- --- -->
<!-- 
## Conclusion

This repository demonstrates a basic yet extensible ROS 2 system. The implementation and documentation aim to provide a clear understanding of the system's functionality and how to reproduce it. Feel free to extend the system or use it as a reference for your own ROS 2 projects. -->
