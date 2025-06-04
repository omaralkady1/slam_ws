#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    use_mock_hardware = LaunchConfiguration('use_mock_hardware')
    
    declare_use_mock = DeclareLaunchArgument(
        'use_mock_hardware',
        default_value='false',
        description='Use mock hardware for testing without real ESP32'
    )
    
    # Get package paths
    pkg_share = FindPackageShare('ros2_control_test').find('ros2_control_test')
    
    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_share, 'urdf', 'test_robot.urdf.xacro'), ' ',
        'use_mock_hardware:=', use_mock_hardware
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Controller manager
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            os.path.join(pkg_share, 'config', 'test_controllers.yaml'),
            os.path.join(pkg_share, 'config', 'hardware_config.yaml')
        ],
        output='screen',
    )
    
    # Spawn controllers
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Delay diff_drive after joint_state_broadcaster
    delay_diff_drive_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    
    # micro-ROS agent (only for real hardware)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['udp4', '--port', '8888'],  # Or 'serial' if using USB
        output='screen',
        condition=LaunchConfiguration('use_mock_hardware', default='false')
    )
    
    # Joint state monitor
    joint_state_monitor = Node(
        package='ros2_control_test',
        executable='joint_state_monitor.py',
        name='joint_state_monitor',
        output='screen',
    )
    
    # RViz
    rviz_config_file = os.path.join(pkg_share, 'config', 'test_view.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen',
    )
    
    return LaunchDescription([
        declare_use_mock,
        robot_state_publisher,
        controller_manager,
        joint_state_broadcaster_spawner,
        delay_diff_drive_spawner,
        micro_ros_agent,
        joint_state_monitor,
        rviz
    ])