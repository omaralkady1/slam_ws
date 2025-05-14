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
find_package(controller_manager_msgs REQUIRED)  # For diagnostics

# NEW: Add hardware interface dependencies
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
  
  DESTINATION lib/${PROJECT_NAME}
)

# NEW: Add hardware interface library
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
  hardware_interface hardware_interface/my_slam_pkg_hardware_interface.xml
)

# Install all configuration files
install(DIRECTORY
  launch
  config
  maps
  worlds
  urdf
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