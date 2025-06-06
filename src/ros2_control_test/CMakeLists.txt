
cmake_minimum_required(VERSION 3.8)
project(ros2_control_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Hardware interface library
add_library(
  ${PROJECT_NAME}_hardware_interface
  SHARED
  hardware_interface/src/esp32_hardware_interface.cpp
)

target_include_directories(
  ${PROJECT_NAME}_hardware_interface
  PRIVATE
  hardware_interface/include
)

ament_target_dependencies(
  ${PROJECT_NAME}_hardware_interface
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  geometry_msgs
  sensor_msgs
)

# Install hardware interface library
install(
  TARGETS ${PROJECT_NAME}_hardware_interface
  DESTINATION lib
)

# Install hardware interface headers
install(
  DIRECTORY hardware_interface/include/
  DESTINATION include
)

# Install hardware interface plugin description
pluginlib_export_plugin_description_file(
  hardware_interface hardware_interface/ros2_control_test_hardware.xml
)

# Install Python scripts
install(PROGRAMS
  scripts/motor_test.py
  scripts/joint_state_monitor.py
  scripts/hardware_diagnostics.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, configs, and URDF
install(DIRECTORY
  launch
  config
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_hardware_interface)
ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()