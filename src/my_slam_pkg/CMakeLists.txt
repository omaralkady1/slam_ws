cmake_minimum_required(VERSION 3.8)
project(my_slam_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

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

# Install Python scripts
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
  
  DESTINATION lib/${PROJECT_NAME}
)

# Install all other files
install(DIRECTORY
  launch
  config
  maps
  worlds
  urdf
  DESTINATION share/${PROJECT_NAME}
)
ament_package()