from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os

def generate_launch_description():
    # Declare arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',  # Changed to true by default for simulation
        description='Use simulation (Gazebo) clock if true'
    )
    
    # Package share directory
    pkg_share = FindPackageShare('my_slam_pkg').find('my_slam_pkg')
    
    # Launch twist_mux
    twist_mux_config = os.path.join(pkg_share, 'config', 'twist_mux.yaml')
    
    twist_mux_node = Node(
        package='twist_mux',
        executable='twist_mux',
        name='twist_mux',
        parameters=[twist_mux_config, {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('/cmd_vel_out', '/diff_drive_controller/cmd_vel')  # Make sure this matches your diff_drive_controller's input
        ],
        output='screen'
    )
    
    return LaunchDescription([
        declare_use_sim_time,
        twist_mux_node
    ])