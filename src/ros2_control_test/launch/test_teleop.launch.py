#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop',
            prefix='xterm -e',
            output='screen',
            remappings=[('/cmd_vel', '/diff_drive_controller/cmd_vel')],
        ),
        
        Node(
            package='ros2_control_test',
            executable='motor_test.py',
            name='motor_test',
            output='screen',
        )
    ])