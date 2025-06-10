#!/usr/bin/env python3

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Declare arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            'use_fake_hardware',
            default_value='false',
            description='Start robot with fake hardware mirroring command to its states',
        )
    )
    
    # Get package directory
    pkg_share = FindPackageShare('my_slam_pkg').find('my_slam_pkg')
    
    # Choose the appropriate URDF file based on the use_fake_hardware parameter
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    
    # Get URDF via xacro
    robot_description_content = Command([
        FindExecutable(name='xacro'), ' ',
        os.path.join(pkg_share, 'urdf', 'car.urdf.xacro'), ' ',
        'use_fake_hardware:=', use_fake_hardware
    ])
    
    robot_description = {'robot_description': robot_description_content}
    
    # Controller configuration
    controller_manager_config = os.path.join(
        pkg_share, 
        'config', 
        'hardware_controllers.yaml'
    )
    
    # Start ROS2 Control node
    controller_manager = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            robot_description,
            controller_manager_config
        ],
        output='screen',
    )
    
    # Start joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Start diff drive controller
    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    
    # Make sure joint_state_broadcaster starts before diff_drive_controller
    diff_drive_controller_spawner_delay = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )
    
    # Robot state publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description],
    )
    
    # Twist multiplexer
    twist_mux_params = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    twist_mux = Node(
        package='twist_mux',
        executable='twist_mux',
        parameters=[twist_mux_params, {'use_sim_time': use_fake_hardware}],
        remappings=[
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')
        ],
        output='screen',
    )
    
    # Static transform for base_footprint to base_link
    static_tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        arguments=['0', '0', '0', '0', '0', '0', 'base_footprint', 'base_link'],
    )
    
    # Optional: micro-ROS agent for real hardware (only launched when not using fake hardware)
    micro_ros_agent = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyUSB0', '--baud', '115200'],
        output='screen',
        condition=UnlessCondition(use_fake_hardware)
    )
    
    # Create the launch description with all declared arguments
    ld = LaunchDescription(declared_arguments)
    
    # Add all nodes to the launch description
    ld.add_action(robot_state_publisher)
    ld.add_action(controller_manager)
    ld.add_action(joint_state_broadcaster_spawner)
    ld.add_action(diff_drive_controller_spawner_delay)
    ld.add_action(twist_mux)
    ld.add_action(static_tf_base_footprint_to_base_link)
    
    # Only add the micro-ROS agent for real hardware
    ld.add_action(micro_ros_agent)
    
    return ld