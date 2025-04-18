cmake_minimum_required(VERSION 3.8)
project(sensor_controller_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(interfaces REQUIRED)  

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(sensor_controller_client src/sensor_controller_client.cpp)
ament_target_dependencies(sensor_controller_client rclcpp std_msgs example_interfaces interfaces)

install(TARGETS
  sensor_controller_client
  DESTINATION lib/${PROJECT_NAME})

add_executable(sensor_controller_server src/sensor_controller_server.cpp)
ament_target_dependencies(sensor_controller_server rclcpp example_interfaces)

install(TARGETS
  sensor_controller_server
  DESTINATION lib/${PROJECT_NAME})

  add_executable(data_publisher src/data_publisher.cpp)
ament_target_dependencies(data_publisher rclcpp std_msgs)

install(TARGETS
  data_publisher
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
