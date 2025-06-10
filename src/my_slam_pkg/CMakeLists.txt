cmake_minimum_required(VERSION 3.8)
project(my_slam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find required packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(slam_toolbox REQUIRED)
find_package(controller_manager REQUIRED)
find_package(ros2_controllers REQUIRED)
find_package(diff_drive_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(twist_mux REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)

# Hardware interface dependencies
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# Install Python scripts with proper permissions
install(PROGRAMS
  scripts/nav_goal_sender.py
  scripts/send_waypoints.py
  scripts/nav_goal_listener.py
  scripts/nav_goal_client.py
  scripts/nav_visualization.py
  scripts/navigate_to_coordinates.py
  scripts/set_initial_pose.py
  scripts/test_map_server.py
  scripts/nav_test.py
  scripts/setup_navigation.py
  scripts/ensure_nav_active.py
  scripts/test_motor_control.py
  scripts/manual_motor_test.py
  scripts/mock_esp32.py
  scripts/rplidar_diagnostics.py
  scripts/setup_rplidar.py
  scripts/test_motors.py
  scripts/test_rplidar.py
  scripts/tf_monitor.py
  scripts/simple_motor_test.py
  scripts/esp32_diagnostics.py  # NEW: ESP32 diagnostics
  scripts/esp32_hardware_controller.py  # NEW: ESP32 hardware controller with IMU
  scripts/slam_diagnostic.py
  scripts/slam_diagnostics.py
  scripts/slam_odometry_calibrator.py
  scripts/imu_enhanced_odometry_node.py  # NEW: IMU-enhanced odometry
  scripts/imu_diagnostics.py
  scripts/ekf_diagnostics.py
  scripts/ekf_test_script.py
  scripts/test_encoders.py
  scripts/velocity_monitor.py
  scripts/flash_esp32_firmware.sh
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
install(DIRECTORY
  my_slam_pkg/
  DESTINATION lib/python3/dist-packages/
  FILES_MATCHING PATTERN "*.py"
)

# Add hardware interface library
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
  nav_msgs
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
  hardware_interface hardware_interface/my_slam_pkg_hardware_interface.xml
)

# Install all configuration files
install(DIRECTORY
  launch
  config
  maps
  worlds
  urdf
  rviz
  firmware_reference
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}_hardware_interface
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()